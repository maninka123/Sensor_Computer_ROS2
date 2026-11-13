#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /workspace/ros2/install/setup.bash
set -u
export PYTHONUNBUFFERED=1
# ARM64 static TLS workaround, discovered from the installed wheel.
for lib in /opt/inference/lib/python*/site-packages/torch.libs/libgomp*.so*; do
  if [[ -f "$lib" ]]; then export LD_PRELOAD="$lib${LD_PRELOAD:+:$LD_PRELOAD}"; break; fi
done
mkdir -p "$HOME/.config/spinnaker"
if [[ ! -f "$HOME/.config/spinnaker/privacy.xml" ]]; then
  printf '%s\n' '<PrivacyConfig><Disabled>true</Disabled></PrivacyConfig>' > "$HOME/.config/spinnaker/privacy.xml"
fi
hardware="${HARDWARE:-true}"
run_child() {
  # Bash background jobs otherwise inherit SIGINT=ignore. Restore it before
  # exec so ROS Python workers and launch shut down cleanly on Docker stop.
  exec python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' "$@"
}
run_child ros2 launch node_pc pipeline.launch.py hardware:="$hardware" &
pipeline=$!
health=""
cleanup() {
  trap - EXIT INT TERM
  [[ -z "$health" ]] || kill -INT "$health" 2>/dev/null || true
  kill -INT "$pipeline" 2>/dev/null || true
  wait "$pipeline" 2>/dev/null || true
  [[ -z "$health" ]] || wait "$health" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
if [[ "$hardware" == true ]]; then
  run_child ros2 run node_pc healthcheck.py --startup-timeout 120 --max-age 20 \
    --bridge-url "ws://${ROSBRIDGE_ADDRESS:-10.20.0.61}:${ROSBRIDGE_PORT:-9090}" \
    --storage-path "$HOME" &
  health=$!
fi
# Any process exiting, including with success, means supervision must restart
# the full container. SIGINT from systemd/docker is handled by cleanup.
if [[ -n "$health" ]]; then wait -n "$pipeline" "$health" || true; else wait "$pipeline" || true; fi
exit 1
