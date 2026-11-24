#!/usr/bin/env bash
# Installed root-owned. Containerized application always runs as uid/gid 1001.
set -euo pipefail
source /etc/sensor-computer/host.env
required=(10.20.0.61 192.168.1.50)
deadline=$((SECONDS+60))
while true; do
  ready=true
  [[ "$(cat /sys/class/net/end1/carrier 2>/dev/null || true)" == 1 ]] || ready=false
  for address in "${required[@]}"; do
    if ! ip -4 -o address show dev end1 | grep -Fq " $address/"; then ready=false; fi
  done
  if [[ "$ready" == true ]]; then break; fi
  if (( SECONDS >= deadline )); then echo 'Sensor Ethernet link or addresses are missing on end1; retrying through systemd.' >&2; exit 1; fi
  sleep 2
done
available=$(df -Pk "$WORKSPACE" | awk 'NR==2 {print $4}')
(( available >= 2097152 )) || { echo 'Less than 2 GiB free; refusing to create acquisition logs.' >&2; exit 1; }
if [[ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null || true)" != yes ]]; then
  echo 'WARNING: host clock is not NTP synchronized. Relative sensor matching may work; absolute timestamps are unverified.' >&2
fi
docker image inspect sensor-computer:jazzy-sdk >/dev/null
[[ -r "$WORKSPACE/ros2/install/node_pc/share/node_pc/local_setup.bash" ]] || {
  echo 'Build is missing; run ros2/tools/build.sh first.' >&2; exit 1;
}
if docker inspect sensor-computer-jazzy >/dev/null 2>&1; then
  owner=$(docker inspect -f '{{index .Config.Labels "io.sensor-computer.managed-by"}}' sensor-computer-jazzy)
  if [[ "$owner" != systemd ]]; then
    echo 'Container name is used by a manual deployment; refusing to replace it.' >&2; exit 1
  fi
  if [[ "$(docker inspect -f '{{.State.Running}}' sensor-computer-jazzy)" == true ]]; then
    echo 'Reattaching the existing systemd-owned container after a supervisor/daemon restart.'
    exec docker attach sensor-computer-jazzy
  fi
  docker rm sensor-computer-jazzy >/dev/null
fi
exec docker run --rm --name sensor-computer-jazzy --init --network host \
  --label io.sensor-computer.managed-by=systemd \
  --user 1001:1001 --cap-drop ALL --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,size=268435456 --tmpfs /run:rw,nosuid,size=16777216 \
  --memory 5g --memory-swap 6g --pids-limit 512 --ulimit core=67108864 \
  --log-driver local --log-opt max-size=10m --log-opt max-file=3 \
  --stop-signal SIGINT --stop-timeout 25 \
  --env-file /etc/sensor-computer/runtime.env -e HOME=/home/rock \
  --mount "type=bind,source=$WORKSPACE,target=/workspace,readonly" \
  --mount "type=bind,source=$WORKSPACE/ros2/state/home,target=/home/rock" \
  -w /home/rock sensor-computer:jazzy-sdk \
  bash /workspace/ros2/tools/container-entrypoint.sh
