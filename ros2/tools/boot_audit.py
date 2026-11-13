#!/usr/bin/env python3
"""Persist an honest post-boot infrastructure and live-stream audit."""
from pathlib import Path
import datetime
import json
import os
import subprocess
import tempfile


def command(*args,timeout=10):
    try:
        result=subprocess.run(args,text=True,capture_output=True,timeout=timeout)
        return result.returncode,result.stdout.strip()
    except (OSError,subprocess.TimeoutExpired) as exc:
        return 1,str(exc)


def main():
    destination=Path('/var/lib/sensor-computer/boot-audit.json')
    try:
        previous=json.loads(destination.read_text())
    except (OSError,ValueError):
        previous={}
    result={'timestamp_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            'previous_boot_id':previous.get('boot_id'),
            'uptime_s':float(Path('/proc/uptime').read_text().split()[0])}
    result['docker_active']=command('systemctl','is-active','docker.service')[0]==0
    result['pipeline_enabled']=command('systemctl','is-enabled','sensor-computer-jazzy.service')[0]==0
    result['pipeline_state']=command('systemctl','show','sensor-computer-jazzy.service','-p','ActiveState','--value')[1]
    result['saved_sensor_addresses']=command('nmcli','-g','ipv4.addresses','connection','show','sensor-computer')[1]
    result['active_sensor_addresses']=command('ip','-4','-o','address','show','dev','end1')[1]
    result['clock_synchronized']=command('timedatectl','show','-p','NTPSynchronized','--value')[1]=='yes'
    result['live_streams_ok']=False
    running=command('docker','inspect','-f','{{.State.Running}}','sensor-computer-jazzy')[1]=='true'
    if running:
        code,output=command('docker','exec','sensor-computer-jazzy','bash','-c',
            'source /opt/ros/jazzy/setup.bash && source /workspace/ros2/install/setup.bash && '
            'exec ros2 run node_pc healthcheck.py --once --startup-timeout 20 --max-age 5 '
            '--storage-path "$HOME" --bridge-url "ws://${ROSBRIDGE_ADDRESS}:${ROSBRIDGE_PORT}"',timeout=35)
        result['live_streams_ok']=code==0
        result['live_check_output']=output
    else:
        result['live_check_output']='No running pipeline container; sensor link/data are not ready.'
    result['infrastructure_ok']=(result['docker_active'] and result['pipeline_enabled'] and
        '10.20.0.61/24' in result['saved_sensor_addresses'] and
        '192.168.1.50/24' in result['saved_sensor_addresses'])
    destination.parent.mkdir(parents=True,exist_ok=True)
    fd,temporary=tempfile.mkstemp(dir=destination.parent,prefix='.boot-audit-')
    try:
        with os.fdopen(fd,'w') as stream:
            json.dump(result,stream,indent=2)
            stream.write('\n');stream.flush();os.fsync(stream.fileno())
        os.replace(temporary,destination)
        directory=os.open(destination.parent,os.O_RDONLY)
        try: os.fsync(directory)
        finally: os.close(directory)
    finally:
        Path(temporary).unlink(missing_ok=True)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['infrastructure_ok'] else 1)


if __name__=='__main__':
    main()
