#!/usr/bin/env bash
# Run with sudo after reviewing/approving this dedicated sensor-port change.
set -euo pipefail
[[ "${1:-}" == --apply ]] || {
  echo 'Plan: create NetworkManager profile sensor-computer on end1, 10.20.0.61/24 and 192.168.1.50/24.'
  echo 'No gateway or DNS; never-default; MTU 1500; duplicate-address detection 3s; autoconnect enabled.'
  echo 'Keep both original profiles and the active USB internet interface unchanged.'
  echo 'Apply: sudo bash ros2/tools/configure_network.sh --apply'
  echo 'Rollback: sudo nmcli connection delete sensor-computer; sudo nmcli connection up "Wired connection 1"'
  exit 0
}
[[ "$EUID" == 0 ]] || { echo 'Run with sudo' >&2; exit 1; }
interface=end1
profile=sensor-computer
[[ -e /sys/class/net/$interface ]] || { echo 'end1 is absent' >&2; exit 1; }
# Refuse to touch a port currently carrying a default route.
if ip -4 route show default | grep -q "dev $interface\b"; then
  echo 'Refusing: end1 currently carries a default route. Review locally before changing it.' >&2
  exit 1
fi
backup="/var/backups/sensor-computer/network-$(date -u +%Y%m%dT%H%M%SZ)"
install -d -m 0700 "$backup"
cp -a /etc/NetworkManager/system-connections "$backup/"
ip -j address > "$backup/addresses.json"
ip -j route > "$backup/routes.json"
created=false
previous_uuid=$(nmcli -g GENERAL.CON-UUID device show "$interface" 2>/dev/null || true)
if ! nmcli -g NAME connection show | grep -Fxq "$profile"; then
  nmcli connection add type ethernet ifname "$interface" con-name "$profile" \
    connection.autoconnect no ipv4.method manual ipv4.addresses '10.20.0.61/24,192.168.1.50/24'
  created=true
else
  existing_interface=$(nmcli -g connection.interface-name connection show "$profile")
  [[ "$existing_interface" == "$interface" ]] || {
    echo 'Refusing: the existing sensor-computer profile belongs to another interface.' >&2
    exit 1
  }
fi
profile_uuid=$(nmcli -g connection.uuid connection show "$profile")
# This profile has no effect on the USB internet connection or original end1 profile.
nmcli connection modify "$profile" connection.interface-name "$interface" \
  connection.autoconnect yes connection.autoconnect-priority 100 \
  connection.autoconnect-retries 0 802-3-ethernet.mtu 1500 \
  ipv4.method manual ipv4.addresses '10.20.0.61/24,192.168.1.50/24' \
  ipv4.gateway '' ipv4.dns '' ipv4.never-default yes ipv4.ignore-auto-dns yes \
  ipv4.dad-timeout 3000 ipv4.may-fail no ipv6.method disabled
if [[ "$(cat /sys/class/net/$interface/carrier 2>/dev/null || echo 0)" == 1 ]]; then
  # A retry can modify an existing static profile. Preserve that original
  # profile on failure instead of deleting it and assuming DHCP still exists.
  if ! nmcli --wait 30 connection up "$profile"; then
    if [[ "$created" == true ]]; then
      nmcli connection delete "$profile"
    else
      python3 - "$backup/system-connections" "$profile_uuid" <<'PY'
import configparser
import os
from pathlib import Path
import sys
import tempfile
for source in Path(sys.argv[1]).iterdir():
    if not source.is_file():
        continue
    config=configparser.ConfigParser(interpolation=None)
    config.read(source)
    if config.get('connection','uuid',fallback='')!=sys.argv[2]:
        continue
    destination=Path('/etc/NetworkManager/system-connections')/source.name
    fd,temporary=tempfile.mkstemp(dir=destination.parent,prefix='.sensor-rollback-')
    try:
        with os.fdopen(fd,'wb') as stream:
            stream.write(source.read_bytes());stream.flush();os.fsync(stream.fileno())
        os.replace(temporary,destination)
    finally:
        Path(temporary).unlink(missing_ok=True)
    break
else:
    raise SystemExit('Original profile was not found in the backup; preserve the backup for local recovery.')
PY
      nmcli connection reload
    fi
    if [[ -n "$previous_uuid" && "$previous_uuid" != -- ]]; then
      nmcli --wait 15 connection up uuid "$previous_uuid" || true
    fi
    echo "Activation failed; original profiles retained/restored from $backup." >&2
    exit 1
  fi
else
  echo 'No sensor cable detected. Profile saved; NetworkManager will activate it when connected.'
fi
echo "Network backup: $backup"
ip -br address
ip route
