#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/../.." && pwd)
if [[ ! -r /var/run/docker.sock || ! -w /var/run/docker.sock ]]; then
  printf -v command '%q ' "$0" "$@"
  exec sg docker -c "$command"
fi
local_mode=false
if [[ "${1:-}" == --local ]]; then local_mode=true; shift; fi
opts=(--rm --network host --user "$(id -u):$(id -g)" -e HOME=/home/rock
      -v "$root:/workspace:ro" -v "$root/ros2/state/home:/home/rock" -w /workspace/ros2)
if [[ -t 0 && -t 1 ]]; then opts+=(-it); fi
if [[ "$local_mode" == true ]]; then
  opts+=(-e ROS_DOMAIN_ID=73 -e CYCLONEDDS_URI=file:///workspace/ros2/deploy/cyclonedds-test.xml)
else
  opts+=(--env-file "$root/ros2/deploy/runtime.env")
fi
if [[ "${1:-}" == shell || $# == 0 ]]; then
  exec docker run "${opts[@]}" sensor-computer:jazzy-sdk bash -c \
    'source /opt/ros/jazzy/setup.bash; source install/setup.bash; exec bash'
fi
exec docker run "${opts[@]}" sensor-computer:jazzy-sdk bash -c \
  'source /opt/ros/jazzy/setup.bash; source install/setup.bash; exec ros2 "$@"' bash "$@"
