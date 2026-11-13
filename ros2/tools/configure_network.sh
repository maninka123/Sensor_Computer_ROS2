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
if ! nmcli -g NAME connection show | grep -Fxq "$profile"; then
  nmcli connection add type ethernet ifname "$interface" con-name "$profile" \
    connection.autoconnect no ipv4.method manual ipv4.addresses '10.20.0.61/24,192.168.1.50/24'
else
  existing_interface=$(nmcli -g connection.interface-name connection show "$profile")
  [[ "$existing_interface" == "$interface" ]] || {
    echo 'Refusing: the existing sensor-computer profile belongs to another interface.' >&2
    exit 1
  }
fi
# This profile has no effect on the USB internet connection or original end1 profile.
nmcli connection modify "$profile" connection.interface-name "$interface" \
  connection.autoconnect yes connection.autoconnect-priority 100 \
  connection.autoconnect-retries 0 802-3-ethernet.mtu 1500 \
  ipv4.method manual ipv4.addresses '10.20.0.61/24,192.168.1.50/24' \
  ipv4.gateway '' ipv4.dns '' ipv4.never-default yes ipv4.ignore-auto-dns yes \
  ipv4.dad-timeout 3000 ipv4.may-fail no ipv6.method disabled
if [[ "$(cat /sys/class/net/$interface/carrier 2>/dev/null || echo 0)" == 1 ]]; then
  # The old end1 profile is retained. A failed activation removes only our new
  # profile and restores original DHCP; it cannot remove the USB default route.
  if ! nmcli --wait 30 connection up "$profile"; then
    nmcli connection delete "$profile"
    nmcli --wait 10 connection up 'Wired connection 1' || true
    echo 'Activation failed; new profile removed and original end1 profile restored.' >&2
    exit 1
  fi
else
  echo 'No sensor cable detected. Profile saved; NetworkManager will activate it when connected.'
fi
echo "Network backup: $backup"
ip -br address
ip route
