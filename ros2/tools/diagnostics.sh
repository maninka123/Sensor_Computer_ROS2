#!/usr/bin/env bash
set -uo pipefail
root=$(cd -- "$(dirname -- "$0")/../.." && pwd)
date -u
uname -a
ip -br address
ip route
nmcli -f NAME,TYPE,DEVICE connection show
systemctl status sensor-computer-jazzy.service --no-pager || true
timedatectl status
df -h /
free -h
for zone in /sys/class/thermal/thermal_zone*; do
  echo "$(cat "$zone/type"): $(cat "$zone/temp") millidegrees C"
done
sg docker -c 'docker ps -a --filter name=sensor-computer-jazzy; docker image inspect -f "{{.Architecture}} {{.Os}} {{.Id}}" sensor-computer:jazzy-sdk'
for address in 10.20.0.22 192.168.1.125 10.20.0.10; do
  ping -c 1 -W 1 "$address" || true
done
echo 'Service logs: sudo journalctl --namespace=sensor-computer -u sensor-computer-jazzy.service -n 150'
echo 'Live stream check: ./ros2/tools/ros.sh run node_pc healthcheck.py --once'
echo 'Status monitor: ./ros2/tools/ros.sh run node_pc monitor.py'
