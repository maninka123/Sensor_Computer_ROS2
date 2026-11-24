#!/usr/bin/env python3
"""Persist live readiness and compare a requested reboot with its saved baseline."""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import os
import shlex
import socket
import subprocess
import tempfile

STATE = Path('/var/lib/sensor-computer')


def command(*args, timeout=10):
    try:
        result = subprocess.run(args, text=True, capture_output=True, timeout=timeout)
        return result.returncode, result.stdout.strip()
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 1, str(exc)


def atomic_json(destination, value):
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(dir=destination.parent, prefix='.audit-')
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
        directory = os.open(destination.parent, os.O_RDONLY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        Path(temporary).unlink(missing_ok=True)


def configuration_snapshot():
    assignments = shlex.split(Path('/etc/sensor-computer/host.env').read_text())
    workspace = Path(next(v.split('=', 1)[1] for v in assignments if v.startswith('WORKSPACE=')))
    paths = [Path(p) for p in [
        '/etc/sensor-computer/runtime.env', '/etc/sensor-computer/host.env',
        '/etc/systemd/system/sensor-computer-jazzy.service',
        '/etc/sysctl.d/90-sensor-computer-dds.conf',
        '/etc/NetworkManager/system-connections/sensor-computer.nmconnection',
        '/etc/gdm3/daemon.conf',
    ]] + [workspace/p for p in [
        'ros2/deploy/cyclonedds.xml', 'ros2/src/node_pc/config/pipeline.yaml',
        'ros2/install/node_pc/share/node_pc/config/pipeline.yaml',
        'ros2/install/node_pc/share/node_pc/config/livox.json',
    ]]
    snapshot = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    return workspace, snapshot


def port_open(address, port):
    try:
        with socket.create_connection((address, port), timeout=1):
            return True
    except OSError:
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--prepare-reboot', action='store_true',
                        help='Save a healthy baseline; does not reboot the host')
    args = parser.parse_args()
    destination = STATE/'boot-audit.json'
    result = {
        'timestamp_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        'uptime_s': float(Path('/proc/uptime').read_text().split()[0]),
    }
    for name, unit in [('docker', 'docker.service'), ('ssh', 'ssh.service'),
                       ('pipeline', 'sensor-computer-jazzy.service')]:
        result[name+'_active'] = command('systemctl', 'is-active', unit)[0] == 0
        result[name+'_enabled'] = command('systemctl', 'is-enabled', unit)[0] == 0
    result['saved_sensor_addresses'] = command('nmcli', '-g', 'ipv4.addresses', 'connection', 'show', 'sensor-computer')[1]
    result['active_sensor_addresses'] = command('ip', '-4', '-o', 'address', 'show', 'dev', 'end1')[1]
    result['sensor_carrier'] = command('cat', '/sys/class/net/end1/carrier')[1] == '1'
    result['clock_synchronized'] = command('timedatectl', 'show', '-p', 'NTPSynchronized', '--value')[1] == 'yes'
    result['ssh_port_open'] = port_open('10.20.0.61', 22)
    result['rosbridge_port_open'] = port_open('10.20.0.61', 9090)
    result['managed_containers'] = command('docker', 'ps', '--filter',
        'label=io.sensor-computer.managed-by=systemd', '--format', '{{.Names}}')[1].splitlines()
    result['image_id'] = command('docker', 'inspect', '-f', '{{.Image}}', 'sensor-computer-jazzy')[1]
    result['live_streams_ok'] = False
    if command('docker', 'inspect', '-f', '{{.State.Running}}', 'sensor-computer-jazzy')[1] == 'true':
        code, output = command('docker', 'exec', 'sensor-computer-jazzy', 'bash', '-c',
            'source /opt/ros/jazzy/setup.bash && source /workspace/ros2/install/setup.bash && '
            'exec ros2 run node_pc healthcheck.py --once --startup-timeout 20 --max-age 5 '
            '--storage-path "$HOME" --bridge-url "ws://${ROSBRIDGE_ADDRESS}:${ROSBRIDGE_PORT}"', timeout=35)
        result['live_streams_ok'] = code == 0
        result['live_check_output'] = output
    try:
        workspace, snapshot = configuration_snapshot()
        result['configuration_hashes'] = snapshot
        code, output = command('python3', str(workspace/'ros2/tools/acceptance_stamp.py'), 'check')
        result['software_acceptance_ok'] = code == 0
        result['software_acceptance_output'] = output
    except (OSError, ValueError, StopIteration) as exc:
        result['configuration_error'] = str(exc)
        result['software_acceptance_ok'] = False
    result['infrastructure_ok'] = all(result[name+'_active'] and result[name+'_enabled']
        for name in ['docker', 'ssh', 'pipeline']) and result['ssh_port_open'] and result['sensor_carrier'] and (
        result['managed_containers'] == ['sensor-computer-jazzy']) and all(
        address in result['saved_sensor_addresses'] and address in result['active_sensor_addresses']
        for address in ['10.20.0.61/24', '192.168.1.50/24'])
    result['overall_ok'] = (result['infrastructure_ok'] and result['live_streams_ok'] and
        result['rosbridge_port_open'] and result['software_acceptance_ok'])
    pending = STATE/'reboot-test-baseline.json'
    if args.prepare_reboot:
        if not result['overall_ok']:
            atomic_json(destination, result)
            print(json.dumps(result, indent=2))
            raise SystemExit('Reboot baseline refused: finish the failing readiness checks first.')
        atomic_json(pending, result)
        result['reboot_test'] = {'state': 'prepared; awaiting a different boot ID', 'passed': False}
    elif pending.exists():
        try:
            before = json.loads(pending.read_text())
            changed = before['boot_id'] != result['boot_id']
            configuration_same = before['configuration_hashes'] == result.get('configuration_hashes')
            image_same = before['image_id'] == result['image_id']
            result['reboot_test'] = {
                'previous_boot_id': before['boot_id'], 'boot_id_changed': changed,
                'configuration_unchanged': configuration_same, 'image_unchanged': image_same,
                'passed': bool(changed and configuration_same and image_same and result['overall_ok']),
            }
        except (OSError, ValueError, KeyError) as exc:
            result['reboot_test'] = {'passed': False, 'error': str(exc)}
    atomic_json(destination, result)
    atomic_json(STATE/'boot-history'/(result['boot_id']+'.json'), result)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['overall_ok'] else 1)


if __name__ == '__main__':
    main()
