#!/usr/bin/env bash
# Install disabled units by default. --enable is a separately approved action.
set -euo pipefail
[[ "$EUID" == 0 ]] || { echo 'Run with sudo' >&2; exit 1; }
ws=$(cd -- "$(dirname -- "$0")/../.." && pwd)
[[ "$(id -u rock)" == 1001 && "$(id -g rock)" == 1001 ]] || { echo 'Review deployment UID/GID for this host.' >&2; exit 1; }
[[ "$ws" != *,* ]] || { echo 'Docker bind-mount paths cannot contain commas.' >&2; exit 1; }
mode="${1:---install}"
[[ "$mode" == --install || "$mode" == --enable ]] || { echo 'Use --install or --enable' >&2; exit 2; }
backup="/var/backups/sensor-computer/services-$(date -u +%Y%m%dT%H%M%S%N)"
install -d -m 0700 "$backup"
for path in /etc/systemd/system/sensor-computer-jazzy.service \
    /etc/systemd/system/sensor-computer-boot-audit.service \
    /etc/systemd/system/sensor-computer-boot-audit.timer /etc/sensor-computer \
    /usr/local/lib/sensor-computer /etc/systemd/journald@sensor-computer.conf; do
  if [[ -e "$path" ]]; then
    target="$backup/${path#/}"
    install -d -m 0700 "$(dirname -- "$target")"
    cp -a -- "$path" "$target"
  fi
done
install -d -m 0755 /usr/local/lib/sensor-computer /etc/sensor-computer
install -m 0755 "$ws/ros2/deploy/run-container.sh" /usr/local/lib/sensor-computer/run-container.sh
install -m 0755 "$ws/ros2/deploy/wait-ready.sh" /usr/local/lib/sensor-computer/wait-ready.sh
install -m 0755 "$ws/ros2/deploy/stop-container.sh" /usr/local/lib/sensor-computer/stop-container.sh
install -m 0755 "$ws/ros2/tools/boot_audit.py" /usr/local/lib/sensor-computer/boot_audit.py
install -m 0644 "$ws/ros2/deploy/runtime.env" /etc/sensor-computer/runtime.env
printf 'WORKSPACE=%q\n' "$ws" > /etc/sensor-computer/host.env
chmod 0644 /etc/sensor-computer/host.env
install -m 0644 "$ws/ros2/deploy/sensor-computer-jazzy.service" /etc/systemd/system/
install -m 0644 "$ws/ros2/deploy/sensor-computer-boot-audit.service" /etc/systemd/system/
install -m 0644 "$ws/ros2/deploy/sensor-computer-boot-audit.timer" /etc/systemd/system/
install -m 0644 "$ws/ros2/deploy/journald.conf" /etc/systemd/journald@sensor-computer.conf
systemctl daemon-reload
systemd-analyze verify /etc/systemd/system/sensor-computer-jazzy.service \
  /etc/systemd/system/sensor-computer-boot-audit.service /etc/systemd/system/sensor-computer-boot-audit.timer
if [[ "$mode" == --enable ]]; then
  python3 "$ws/ros2/tools/acceptance_stamp.py" check
  if [[ ! -f "$ws/.local-deploy/live-streams-passed" ]]; then
    echo 'Live hardware acceptance is pending. Startup will wait/retry until sensors and their Ethernet link are available.'
  fi
  systemctl enable docker.service containerd.service sensor-computer-jazzy.service sensor-computer-boot-audit.timer
  systemctl --no-block start sensor-computer-boot-audit.timer
  systemctl --no-block start sensor-computer-jazzy.service
else
  echo 'Service files installed. Boot startup and desktop autologin were not enabled.'
fi
echo "Backup: $backup"
