#!/usr/bin/env bash
# ExecStartPost keeps systemd in 'activating' until real topic data is received.
set -euo pipefail
deadline=$((SECONDS+195))
while (( SECONDS < deadline )); do
  if [[ "$(docker inspect -f '{{.State.Running}}' sensor-computer-jazzy 2>/dev/null || true)" == true ]]; then
    exec docker exec sensor-computer-jazzy bash -c '
      source /opt/ros/jazzy/setup.bash
      source /workspace/ros2/install/setup.bash
      exec ros2 run node_pc healthcheck.py --once --startup-timeout 125 --max-age 20 \
        --storage-path "$HOME" \
        --bridge-url "ws://${ROSBRIDGE_ADDRESS:-10.20.0.61}:${ROSBRIDGE_PORT:-9090}"
    '
  fi
  sleep 2
done
echo 'No running pipeline container became available before the startup deadline.' >&2
exit 1
