# ROCK 5C: Debian 12 host, ROS 2 Jazzy sensor computer

This deployment keeps the Radxa Debian kernel and desktop and runs Ubuntu 24.04
ARM64 / ROS 2 Jazzy inside Docker. The original Catkin/Noetic source remains in the upstream Sensor_Computer
repository and the original local checkout. This ROS 2 repository publishes
the Jazzy deployment separately; do not run legacy Noetic startup scripts here.

Development branch: `deploy/rock5c-debian12-jazzy`.
See [environment and compatibility](../docs/ROCK5C_ENVIRONMENT.md),
[migration details](MIGRATION.md), [transport contract](TRANSPORTS.md), and
[verification results](VERIFICATION.md).

## Installation and reproducibility

Docker is installed from Debian's `docker.io` package. The host has no native
Ubuntu ROS packages or Spinnaker packages. The official ROS ARM64 base image
is pinned by digest in `docker/Dockerfile`. C++ dependencies, ROS packages,
Python 3.12, PyTorch 2.6.0 CPU, ONNX Runtime 1.20.1 and NumPy 1.26.4 are inside
the image. Vendor drivers and the original Avia SDK are pinned by commit in
`tools/prepare_sources.py`; the source checkouts are ignored under `vendor/`.

This machine's downloaded vendor archive is:

```
/home/rock/Downloads/spinnaker-4.4.0.246-noble-arm64-pkg.tar.gz
SHA256 9c4aef63cb070a4734e7e93051898b70aaeac17c13b86ef7cbee8c787554e261
```

Obtain that Noble ARM64 package from Teledyne under its licence. Never use the
original Ubuntu 20.04 / 4.2.0.46 package as a substitute on this image. The
Docker build records acceptance of the supplied SDK's EULA for unattended
package installation; analytics are disabled in the runtime user's config.
Only the GenTL runtime, Spinnaker C++ runtime, and development headers are
installed; desktop SDK utilities and USB/kernel configuration scripts are
unnecessary for this GigE application. SDK archives/libraries are not committed.

From a new local terminal after Docker-group membership is active:

```bash
cd /home/rock/Sensor_Computer
mkdir -p ros2/vendor/sdk
tar -xzf ~/Downloads/spinnaker-4.4.0.246-noble-arm64-pkg.tar.gz -C ros2/vendor/sdk
./ros2/tools/build.sh
./ros2/tools/verify.sh
sg docker -c 'python3 ros2/tools/container_recovery_test.py'
```

`build.sh` limits compilation to two jobs and builds all six ament packages.
Existing build results are specific to ARM64, Noble and this workspace path;
rebuild instead of copying the original ROS 1 `build/` or `devel/` directories.
The resulting images are `sensor-computer:jazzy-dev` and
`sensor-computer:jazzy-sdk`. Development CLI access:

```bash
./ros2/tools/ros.sh --local pkg executables node_pc
./ros2/tools/ros.sh --local shell
```

`--local` selects an isolated loopback test domain; omit it for the sensor LAN.
The wrapper handles this session's not-yet-refreshed Docker group membership.

## Network values for this computer

| Component | Setting |
|---|---|
| Sensor Ethernet interface | `end1` |
| Sensor/server address | `10.20.0.61/24` |
| Livox host address on the same interface | `192.168.1.50/24` |
| FLIR Blackfly S | `10.20.0.22`, serial `24510717` |
| Original Livox Avia | `192.168.1.125`, broadcast code `3JEDLB30015Y251` |
| Central computer | `10.20.0.10/24` |
| ROSBridge | `ws://10.20.0.61:9090` |
| DDS domain for this first ROS 2 rig | `0`, per the supplied Central device configuration |

`deploy/runtime.env` is the Jazzy runtime configuration. It has no ROS 1 master
or TCPROS settings. `deploy/cyclonedds.xml` restricts DDS to `end1`, adds the
Central PC as a discovery peer, and uses 1400-byte RTPS messages for MTU 1500.
ROSBridge binds only to the private sensor address. The USB Ethernet interface
`enx207bd2b1fe36` retains its existing DHCP internet, route and DNS configuration.

The persistent NetworkManager profile has been saved on this board and will
activate when its sensor cable is connected. Reproduce it with this script:

```bash
./ros2/tools/configure_network.sh             # print plan only
sudo bash ros2/tools/configure_network.sh --apply
```

It refuses to modify `end1` if that port carries the current default route,
backs up existing profiles, retains the original profiles, adds both static
addresses, enables duplicate-address detection, and supplies no gateway or DNS.
With no cable, the profile is saved for activation when carrier appears. If a
cabled activation fails, the new profile is removed and the original `end1`
profile is restored. Rollback after a successful configuration:

```bash
sudo nmcli connection delete sensor-computer
sudo nmcli connection up 'Wired connection 1'
```

Do not change the USB internet profile or assign the same sensor subnet to two
interfaces. Additional rigs on a shared switch must use unique sensor/host
addresses, or separate private sensor LANs. Every rig using these absolute ROS 2
topic names must also have its own DDS domain. IP addresses alone do not isolate DDS.

## Unattended operation

Systemd owns the production container lifecycle. The application is independent
of desktop login/logout. ExecStartPost keeps systemd in activating state until
valid, changing camera/LiDAR/IMU/fused samples arrive. Empty frames or a frozen
repeated timestamp cannot establish health. It checks the local addresses, starts acquisition,
processing and ROSBridge, and verifies fresh camera/LiDAR/IMU/fused messages.
It does not wait for the Central computer. Missing streams, bridge handshake
failure, or an essential process exit terminate the container and trigger a
retry after 20 seconds. Startup gets 120 seconds for sensor streams, following
a maximum 60-second address wait. Reconnection and real sensor recovery still
need hardware acceptance testing.

Install service files without enabling startup:

```bash
sudo bash ros2/tools/install_services.sh --install
```

After approval to enable permanent startup:

```bash
sudo bash ros2/tools/install_services.sh --enable
systemctl status sensor-computer-jazzy.service
sudo journalctl --namespace=sensor-computer -u sensor-computer-jazzy.service -f
```

There is one production container name, `sensor-computer-jazzy`. The unit
conflicts with the legacy ROS 1 units and refuses to duplicate a running named
container. Do not use Compose concurrently. `deploy/compose.yaml` is an
alternative manual deployment with automatic restart disabled by default.

The container runs as UID/GID 1001 with no capabilities or Docker socket,
a read-only workspace/image, writable state under `ros2/state/home`, 5 GiB RAM /
6 GiB combined RAM+swap limits, and a 512-process limit. Docker logs are capped
at 30 MiB; the application journal namespace is capped at 256 MiB with maximum
90-day retention. Detailed ROS logs use a bounded ephemeral `/tmp` tmpfs.
Startup and the stream watchdog enforce a 2 GiB free-storage reserve.

Stop or disable startup:

```bash
sudo systemctl stop sensor-computer-jazzy.service
sudo systemctl disable --now sensor-computer-jazzy.service
```

Automatic suspend/hibernate and Plasma idle locking have already been disabled
at the user's explicit request. Autologin is now configured for the next boot. KDE Plasma runs under **GDM**, so autologin uses
the existing GDM configuration, not SDDM. Its separate staged script is:

```bash
./ros2/tools/configure_autologin.sh             # print plan only
sudo bash ros2/tools/configure_autologin.sh --apply
```

It takes effect at the next login/boot and does not restart the current desktop.
The reproducible power policy is `tools/disable_sleep.sh`. Backups are under
`/var/backups/sensor-computer/` and the initial user power settings are in
`../.local-deploy/power-backup/`. SSH authentication settings are unchanged.

## Connect sensors and verify

Connect the powered camera and Avia through the sensor switch to `end1`. The
configured serial/broadcast code must match those physical devices. Once the
approved service is running:

```bash
./ros2/tools/diagnostics.sh
./ros2/tools/live_acceptance.sh
./ros2/tools/ros.sh run node_pc monitor.py
./ros2/tools/ros.sh topic hz /camera/image_raw
./ros2/tools/ros.sh topic hz /livox/lidar
./ros2/tools/ros.sh topic hz /livox/imu
./ros2/tools/ros.sh topic hz /merged_colored_cloud
./ros2/tools/ros.sh topic bw /merged_colored_cloud
```

The monitor subscribes to small status topics. It does not receive bulk images
or intermediate clouds. The SDK enumeration probe source is
`tools/sdk_probe.cpp`; its compiled copy is `state/sdk-probe` on this machine.

Verify 484x366 camera geometry at 35 FPS, approximately 10 Hz LiDAR output,
actual Avia IMU rate, matched-frame counts and colour projection against real
geometry. Calibration is retained from the original rig; moving sensors or using
different optics requires recalibration. Test camera/LiDAR unplug and reconnect,
real Windows delivery/control, repeated approved reboots, and sustained load.
No real hardware, physical-network, Windows, or reboot test has been claimed.

The host currently synchronizes its wall clock using NTP and has no readable
RTC. Its existing systemd-timesyncd clock file now retains a current timestamp
under `/var/lib/systemd/timesync/clock`; it cannot measure time while powered
off. Offline boot and power-loss recovery can therefore have an approximate date;
startup logs a clock warning instead of requiring a remote server. Relative
timestamp conversion recovers after jumps, but absolute timestamps require a
valid clock source. Power-supply quality and causes of earlier restarts remain
unverified; no explicit current brownout/panic/OOM evidence was found.

After boot, inspect `sudo cat /var/lib/sensor-computer/boot-audit.json`. The audit
records infrastructure and live sensor readiness separately. Restart the app with
`sudo systemctl restart sensor-computer-jazzy.service`; Docker restarts also restart
the owning unit. Production uses systemd alone for retries, with no competing
Docker restart policy. The default pipeline does not record bags locally; no
recording durability or physical power-cycle result is claimed.
