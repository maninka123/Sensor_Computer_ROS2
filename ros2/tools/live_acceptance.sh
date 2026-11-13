#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/../.." && pwd)
for address in 10.20.0.61 192.168.1.50; do
  ip -4 -o address show dev end1 | grep -Fq " $address/" || {
    echo "Required address $address is missing on end1. Connect the sensor switch and apply the approved NetworkManager profile." >&2
    exit 1
  }
done
[[ "$(cat /sys/class/net/end1/carrier)" == 1 ]] || { echo 'Sensor Ethernet cable is disconnected.' >&2; exit 1; }
for address in 10.20.0.22 192.168.1.125; do
  ip route get "$address"
  ping -c 1 -W 1 "$address" || true
done
"$root/ros2/tools/ros.sh" run node_pc healthcheck.py --once --startup-timeout 120 \
  --bridge-url ws://10.20.0.61:9090 --storage-path /home/rock
mkdir -p "$root/.local-deploy"
date -u +%FT%TZ > "$root/.local-deploy/live-streams-passed"
echo 'PASS: fresh sensor and fused-output streams, plus local ROSBridge handshake.'
echo 'Still verify real camera image geometry, calibration, sensor rates and the Windows client.'
