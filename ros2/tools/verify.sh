#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")/../.."
if [[ ! -w /var/run/docker.sock ]]; then
  printf -v command '%q ' "$0" "$@"
  exec sg docker -c "$command"
fi
mkdir -p .local-deploy
rm -f .local-deploy/software-tests-passed
python3 -m unittest discover -s ros2/tests -v 2>&1 | tee .local-deploy/deployment-guard-tests.log
docker run --rm --network host --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -e MAKEFLAGS=-j2 -e CMAKE_BUILD_PARALLEL_LEVEL=2 \
  -v "$PWD:/workspace" -w /workspace/ros2 sensor-computer:jazzy-sdk bash -c \
  'source /opt/ros/jazzy/setup.bash && colcon build --base-paths src vendor/livox vendor/flir \
    --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF' \
  > .local-deploy/verification-build.log 2>&1
fingerprint=$(python3 ros2/tools/acceptance_stamp.py fingerprint)
docker run --rm --network host --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -e ROS_DOMAIN_ID=73 \
  -e CYCLONEDDS_URI=file:///workspace/ros2/deploy/cyclonedds-test.xml \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v "$PWD:/workspace" -w /workspace/ros2 \
  sensor-computer:jazzy-sdk bash -c \
  'source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 tools/integration_test.py' \
  2>&1 | tee .local-deploy/integration-tests.log
python3 ros2/tools/container_recovery_test.py 2>&1 | tee .local-deploy/container-recovery-tests.log
python3 ros2/tools/acceptance_stamp.py write --expected "$fingerprint"
