#!/usr/bin/env bash
# Explicitly authorized unattended-operation power policy, for this ROCK 5C.
set -euo pipefail
if [[ "$EUID" != 0 ]]; then exec sudo bash "$0"; fi
backup="/var/backups/sensor-computer/power-$(date -u +%Y%m%dT%H%M%SZ)"
install -d -m 0700 "$backup"
for file in powermanagementprofilesrc powerdevilrc kscreenlockerrc ksmserverrc; do
  [[ ! -f "/home/rock/.config/$file" ]] || cp -a "/home/rock/.config/$file" "$backup/"
done
for directory in /etc/systemd/sleep.conf.d /etc/systemd/logind.conf.d; do
  [[ ! -d "$directory" ]] || cp -a "$directory" "$backup/"
done
systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target suspend-then-hibernate.target
install -d -m 0755 /etc/systemd/sleep.conf.d /etc/systemd/logind.conf.d
cat > /etc/systemd/sleep.conf.d/60-sensor-computer.conf <<'EOF'
[Sleep]
AllowSuspend=no
AllowHibernation=no
AllowSuspendThenHibernate=no
AllowHybridSleep=no
EOF
cat > /etc/systemd/logind.conf.d/60-sensor-computer.conf <<'EOF'
[Login]
IdleAction=ignore
HandleSuspendKey=ignore
HandleHibernateKey=ignore
HandleLidSwitch=ignore
HandleLidSwitchExternalPower=ignore
HandleLidSwitchDocked=ignore
EOF
runuser -u rock -- env DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1001/bus bash <<'USER'
set -e
for profile in AC Battery LowBattery; do
  for key in idleTime suspendType; do
    kwriteconfig5 --file powermanagementprofilesrc --group "$profile" --group SuspendSession --key "$key" 0
  done
  for key in lidAction powerButtonAction powerDownAction; do
    kwriteconfig5 --file powermanagementprofilesrc --group "$profile" --group HandleButtonEvents --key "$key" 0
  done
  kwriteconfig5 --file powermanagementprofilesrc --group "$profile" --group DPMSControl --key idleTime -- -1
  kwriteconfig5 --file powermanagementprofilesrc --group "$profile" --group DimDisplay --key idleTime 0
done
kwriteconfig5 --file powerdevilrc --group BatteryManagement --key BatteryCriticalAction 0
kwriteconfig5 --file kscreenlockerrc --group Daemon --key Autolock --type bool false
kwriteconfig5 --file kscreenlockerrc --group Daemon --key LockOnResume --type bool false
kwriteconfig5 --file kscreenlockerrc --group Daemon --key Timeout 0
qdbus org.kde.Solid.PowerManagement /org/kde/Solid/PowerManagement org.kde.Solid.PowerManagement.reparseConfiguration || true
qdbus org.kde.ksmserver /ScreenSaver org.kde.screensaver.configure || true
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type nothing
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-timeout 0
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type nothing
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-timeout 0
gsettings set org.gnome.settings-daemon.plugins.power power-button-action nothing
gsettings set org.gnome.settings-daemon.plugins.power idle-dim false
gsettings set org.gnome.desktop.session idle-delay 0
gsettings set org.gnome.desktop.screensaver lock-enabled false
gsettings set org.gnome.desktop.screensaver idle-activation-enabled false
gsettings set org.gnome.desktop.screensaver logout-enabled false
USER
systemctl daemon-reload
echo "Power settings backed up in $backup. Active sessions were not restarted."
