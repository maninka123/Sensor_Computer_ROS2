#!/usr/bin/env bash
set -euo pipefail
name=sensor-computer-jazzy
owner=$(docker inspect -f '{{index .Config.Labels "io.sensor-computer.managed-by"}}' "$name" 2>/dev/null || true)
if [[ "$owner" == systemd ]]; then
  docker stop --time 25 "$name"
fi
