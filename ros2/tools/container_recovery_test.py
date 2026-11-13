#!/usr/bin/env python3
"""Temporary isolated-domain test of the production container supervisor."""
from pathlib import Path
import json
import os
import subprocess
import time
import socket

root = Path(__file__).resolve().parents[2]
name = 'sensor-jazzy-recovery-test'
home = root/'.local-deploy/recovery-home'
home.mkdir(exist_ok=True)


def docker(*args, check=True):
    return subprocess.run(['docker', *args], text=True, capture_output=True, check=check)


def wait_bridge(timeout=75):
    deadline = time.monotonic()+timeout
    while time.monotonic()<deadline:
        try:
            with socket.create_connection(('127.0.0.1', 19091), timeout=.5) as connection:
                connection.sendall(b'GET / HTTP/1.1\r\nHost: 127.0.0.1:19091\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\r\n')
                response = connection.recv(4096)
                if not response.startswith(b'HTTP/1.1 101'):
                    raise RuntimeError('WebSocket upgrade failed')
            return
        except Exception:
            time.sleep(.5)
    raise RuntimeError('Test bridge did not become reachable')


if docker('inspect', name, check=False).returncode == 0:
    raise SystemExit('Test container already exists; inspect it before retrying')
try:
    docker('run', '-d', '--name', name, '--init', '--network', 'host',
        '--user', f'{os.getuid()}:{os.getgid()}', '--read-only', '--cap-drop', 'ALL',
        '--security-opt', 'no-new-privileges', '--memory', '5g', '--memory-swap', '6g',
        '--pids-limit', '512', '--tmpfs', '/tmp:rw,nosuid,size=268435456',
        '--tmpfs', '/run:rw,nosuid,size=16777216', '--restart', 'on-failure:2',
        '--stop-signal', 'SIGINT', '--stop-timeout', '25',
        '--log-driver', 'local', '--log-opt', 'max-size=10m', '--log-opt', 'max-file=3',
        '-v', f'{root}:/workspace:ro', '-v', f'{home}:/home/rock', '-w', '/home/rock',
        '-e', 'HOME=/home/rock', '-e', 'HARDWARE=false', '-e', 'ROS_DOMAIN_ID=74',
        '-e', 'CYCLONEDDS_URI=file:///workspace/ros2/deploy/cyclonedds-test.xml',
        '-e', 'ROSBRIDGE_ADDRESS=127.0.0.1', '-e', 'ROSBRIDGE_PORT=19091',
        '-e', 'ROS_LOG_DIR=/tmp/ros-log',
        'sensor-computer:jazzy-sdk', 'bash', '/workspace/ros2/tools/container-entrypoint.sh')
    wait_bridge()
    print('PASS: unprivileged read-only container starts ROS 2 and ROSBridge', flush=True)
    state = json.loads(docker('inspect', name).stdout)[0]
    assert state['HostConfig']['Memory'] == 5*1024**3
    assert state['HostConfig']['PidsLimit'] == 512
    docker('exec', name, 'python3', '-c', '''
from pathlib import Path
import os, signal
for entry in Path('/proc').glob('[0-9]*/cmdline'):
    args = entry.read_bytes().decode(errors='replace').split('\\0')
    if len(args)>1 and args[1].endswith('/rosbridge_websocket'):
        os.kill(int(entry.parent.name), signal.SIGKILL)
        break
else:
    raise SystemExit('Bridge process was not found')
''')
    deadline=time.monotonic()+45
    while time.monotonic()<deadline:
        state=json.loads(docker('inspect', name).stdout)[0]
        if state['RestartCount']>=1:
            break
        time.sleep(.5)
    assert state['RestartCount']>=1, 'Bridge exit did not restart the container'
    wait_bridge()
    print('PASS: killing ROSBridge restarts the container and restores WebSocket service', flush=True)
    benchmark = docker('exec', name, 'bash', '-c',
        'source /opt/ros/jazzy/setup.bash && python3 /workspace/ros2/tools/transport_benchmark.py --output /home/rock/transport-benchmark.json')
    print(benchmark.stdout, flush=True)
    (root/'.local-deploy/transport-benchmark.json').write_text((home/'transport-benchmark.json').read_text())
    docker('stop', '--time', '25', name)
    count=json.loads(docker('inspect', name).stdout)[0]['RestartCount']
    time.sleep(3)
    state=json.loads(docker('inspect', name).stdout)[0]
    assert not state['State']['Running'] and state['RestartCount']==count
    print('PASS: deliberate stop leaves the container stopped', flush=True)
finally:
    logs=docker('logs', name, check=False)
    (root/'.local-deploy/container-recovery-runtime.log').write_text(logs.stdout+logs.stderr)
    docker('stop', '--time', '10', name, check=False)
    docker('rm', name, check=False)
