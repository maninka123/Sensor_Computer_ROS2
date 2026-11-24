#!/usr/bin/env bash
# Build explicit commissioning tools; this does not change any device settings.
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/../.." && pwd)
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$root:/workspace" -w /workspace/ros2 sensor-computer:jazzy-sdk bash -c '
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  set -e
  g++ -std=c++17 tools/camera_network.cpp -I/opt/spinnaker/include \
    -L/opt/spinnaker/lib -Wl,-rpath,/opt/spinnaker/lib -lSpinnaker -o state/camera-network
  g++ -std=c++17 tools/livox_network.cpp -Iinstall/livox_sdk_vendor/include \
    -Linstall/livox_sdk_vendor/lib -llivox_sdk_static -lpthread -o state/livox-network
'
