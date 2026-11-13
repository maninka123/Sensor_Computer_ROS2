#!/usr/bin/env bash
set -euo pipefail
[[ "${1:-}" == --apply ]] || {
  echo 'Plan: enable GDM automatic Plasma login for rock; pipeline startup remains independent.'
  echo 'Apply after approval: sudo bash ros2/tools/configure_autologin.sh --apply'
  exit 0
}
[[ "$EUID" == 0 ]] || { echo 'Run with sudo' >&2; exit 1; }
[[ -f /etc/gdm3/daemon.conf ]] || { echo 'This host does not use GDM; stop and review.' >&2; exit 1; }
backup="/var/backups/sensor-computer/gdm-$(date -u +%Y%m%dT%H%M%SZ).conf"
install -d -m 0700 /var/backups/sensor-computer
cp -a /etc/gdm3/daemon.conf "$backup"
python3 - <<'PY'
from pathlib import Path
import re
p=Path('/etc/gdm3/daemon.conf')
text=p.read_text()
start=text.index('[daemon]')+len('[daemon]')
end=text.find('\n[', start)
end=len(text) if end<0 else end
section=text[start:end]
for key,value in [('AutomaticLoginEnable','true'),('AutomaticLogin','rock')]:
    pattern=r'(?m)^\s*#?\s*'+key+r'\s*=.*$'
    if re.search(pattern,section):
        section=re.sub(pattern, '\n'+key+' = '+value, section, count=1)
    else:
        section+='\n'+key+' = '+value+'\n'
p.write_text(text[:start]+section+text[end:])
PY
echo "GDM configuration updated for the next boot. Current desktop session was not restarted. Backup: $backup"
