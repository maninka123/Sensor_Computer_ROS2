#!/usr/bin/env bash
# Enable SSH with the existing host authentication policy.
set -euo pipefail
[[ "$EUID" == 0 ]] || { echo 'Run with sudo' >&2; exit 1; }
command -v /usr/sbin/sshd >/dev/null || {
  echo 'Install openssh-server first: sudo apt-get install openssh-server' >&2
  exit 1
}
install -d -m 0755 /run/sshd
/usr/sbin/sshd -t
systemctl enable --now ssh.service
systemctl is-active ssh.service
echo 'Connect from a reachable client: ssh rock@10.20.0.61'
