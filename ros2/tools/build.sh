#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")/.."
[[ "$(uname -m)" == aarch64 && "$EUID" != 0 ]] || {
  echo 'Build on this ARM64 board as the deployment user, without sudo.' >&2; exit 1;
}
if [[ ! -w /var/run/docker.sock ]]; then
  printf -v command '%q ' "$0" "$@"
  exec sg docker -c "$command"
fi
python3 tools/prepare_state.py
for package in libgentl libspinnaker libspinnaker-dev; do
  file="vendor/sdk/spinnaker-4.4.0.246-noble-arm64/${package}_4.4.0.246_arm64.deb"
  [[ -f "$file" ]] || { echo "Missing vendor package: $file; see ros2/README.md" >&2; exit 1; }
  [[ "$(dpkg-deb -f "$file" Architecture)" == arm64 && "$(dpkg-deb -f "$file" Version)" == 4.4.0.246 ]] || {
    echo "Wrong SDK architecture/version: $file" >&2; exit 1;
  }
done
python3 tools/prepare_sources.py
docker build --network host -t sensor-computer:jazzy-dev -f docker/Dockerfile .
docker build --network host -t sensor-computer:jazzy-sdk -f docker/Dockerfile.sdk .
docker run --rm --network host --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -e MAKEFLAGS=-j2 -e CMAKE_BUILD_PARALLEL_LEVEL=2 \
  -v "$(cd .. && pwd):/workspace" -w /workspace/ros2 \
  sensor-computer:jazzy-sdk bash -c \
  'source /opt/ros/jazzy/setup.bash && colcon build --base-paths src vendor/livox vendor/flir \
    --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    --event-handlers console_direct+'
printf '%s\n' 'Build finished. Run tools/verify.sh before enabling permanent startup.'
