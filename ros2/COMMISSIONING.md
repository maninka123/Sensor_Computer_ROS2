# ROCK 5C live commissioning — 2026-11-24

The connected hardware differs from the original repository's rig. The camera
was serial `25120439` (BFS-PGE-27S5C), with DHCP enabled and no usable lease,
so it selected `169.254.184.78`. The old camera selection was `24510717`.
The Avia broadcast code is `3JEDN920015M711`; the old whitelist selected
`3JEDLB30015Y251`. Its original static address was `192.168.1.171`.
These were device provisioning/identity mismatches, not a ROS 1 transport issue.

Applied configuration:

| Device | Address / identity |
|---|---|
| ROCK Ethernet `end1` | `10.20.0.61/24` and `192.168.1.50/24` |
| FLIR camera | persistent `10.20.0.22/24`, serial `25120439`, DHCP disabled |
| Livox Avia | static `192.168.1.125/24`, code `3JEDN920015M711` |
| Windows Central | existing `10.20.0.10` peer |
| ROSBridge | `ws://10.20.0.61:9090`, remains available with DDS |

The Avia accepted its static IP and software reboot command; ping to the new
address succeeded. Camera persistent mode/IP/DHCP settings were read back
successfully. Gateway fields use the subnet's `.1` because the devices rejected
zero for persistent gateway; local sensor communication needs no router.
The host sensor profile has no default route. Temporary USB installation
internet is separate and is not required by production startup.

## Live results

The default Linux UDP socket cap was 212,992 bytes and the host recorded 272,029
UDP receive-buffer errors before tuning. The pipeline could receive some raw
clouds but no merged cloud. Increasing the host buffer cap to 32 MiB and asking
Cyclone DDS for a 16 MiB receive buffer restored multi-megabyte cloud delivery.
`deploy/90-sensor-computer-dds.conf` persists the cap; `install_services.sh`
installs and applies it. MTU remains 1500 and RTPS messages remain 1400 bytes.
See [Cyclone DDS socket buffer configuration](https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html).

A 12-second live observation received 121 raw clouds, 121 shifted clouds,
420 camera images and 12 merged/filtered clouds. The latest fused sample
contained 185,767 points (6,687,612 bytes), with all 10 source frames matched.
A best-effort observer received 6 fused samples during a simultaneous large
ROSBridge stream test; this is not a guarantee of lossless delivery. The latest
fused sample was approximately 0.99 seconds old when received, including the
10-scan accumulation and processing. Camera driver reported about 35 FPS and
zero acquisition drops. Image geometry was confirmed as 484×366 Bayer RG8.

The pinned original Livox driver passes through device uptime and acceleration
in g. Hardware launch now exposes that driver IMU as `/livox/imu_raw`, and the
existing timestamp node publishes `/livox/imu` using the LiDAR clock conversion
and SI acceleration (g × 9.80665). Gyroscope values remain rad/s. The latest
corrected IMU was about 45 ms old and acceleration magnitude about 9.76 m/s²
at rest. A real DDS regression covers waiting for the first LiDAR clock anchor,
SI units and a sensor uptime reset. The public IMU and control topic names stay
the same. Raw LiDAR remains device-relative; `/livox/lidar_shifted` and fused
output use the ROS epoch. This is arrival-based clock estimation, not PTP.
See [Livox SDK IMU units](https://github.com/Livox-SDK/Livox-SDK/blob/master/sdk_core/include/livox_def.h).

Same-host clients on the production network configuration received three valid
live clouds through native DDS and three through ROSBridge, plus IMU,
temperature and status. A ROSBridge voxel-size command, status acknowledgement
and restoration passed. An initial combined-socket control test timed out behind
large cloud JSON; the checker now opens a separate control/status socket to
avoid that backlog. Use a separate control socket in Central for responsive
controls while large fallback streams are active. These are ROCK-side client
tests, not verified Windows application reception or Ethernet latency tests.
See the [November 25 follow-up](#windows-follow-up--2026-11-25) for the separate Windows checks.

Seven deployment guard tests, the additional IMU DDS regression, and all nine
pipeline integration tests passed. Evidence is local under
`.local-deploy/live-2026-11-24/` and is excluded from Git.

## Shutdown and recovery correction

Live shutdown initially crashed the pinned original driver. A core backtrace
showed its SIGINT handler calling `exit()` while the SDK discovery thread was
still running. Keeping the ROS 2 signal handler, waking the polling semaphore
and joining the polling thread corrected that teardown ordering. A subsequent
core showed a bundled SDK spdlog/Jazzy spdlog symbol conflict during logger
teardown. Linking the SDK archive with `--exclude-libs,liblivox_sdk_static.a`
keeps its logging symbols private. Time-sync pointers are also initialized and
SDK deinitialization is guarded against repeats. `prepare_sources.py` reproduces
all adaptations and its patch block was checked for idempotence.

The corrected driver was started with the actual connected Avia and stopped by
SIGINT: **exit status 0**, with SDK deinitialization logged. Repeated with the old,
unmatched broadcast code (no sensor connected to that driver): **exit status 0**.
This verifies graceful software shutdown with both present and absent sensors.
The systemd pipeline was then restarted for continued live acquisition. Full
physical reboot/power-cycle validation remains pending.

## SSH and operation

SSH was installed but disabled. It is now enabled at boot and listening on
port 22. Existing-user password authentication succeeded locally using the
sensor address. The Windows host subsequently reached SSH (pre-authentication
disconnect was logged); a successful remote Windows login was not observed.
No root-login or password policy was relaxed.

```bash
ssh rock@10.20.0.61
systemctl is-active ssh sensor-computer-jazzy
sudo journalctl --namespace=sensor-computer -u sensor-computer-jazzy -f
sudo systemctl restart sensor-computer-jazzy
sudo systemctl disable --now sensor-computer-jazzy
```

The remote PC must have a reachable address on `10.20.0.0/24` (the existing
Central address is `10.20.0.10`), with a route through the sensor switch.
`tools/configure_ssh.sh` reproduces service enablement while retaining the host's
existing authentication policy. Startup is independent of desktop login.

## Explicit provisioning tools

Build with `./ros2/tools/build_network_tools.sh`. Stop acquisition first:

```bash
sudo systemctl stop sensor-computer-jazzy
```

Run tools in a host-network SDK container with the workspace mounted at
`/workspace` and source `ros2/install/setup.bash`:

```bash
# Inspect exact device only; no IP change.
/workspace/ros2/state/camera-network 25120439
/workspace/ros2/state/livox-network 3JEDN920015M711
# Explicitly provision the selected device; /24 subnet, gateway subnet.1.
/workspace/ros2/state/camera-network 25120439 10.20.0.22
/workspace/ros2/state/livox-network 3JEDN920015M711 192.168.1.125 --reboot
```

The host needs an address on the camera's current subnet before persistent
configuration can be written. A temporary `169.254.1.1/16` address was used for
initial setup and removed afterward. Do not configure arbitrary discovered
sensors or invoke these tools automatically at boot. Restore acquisition with
`sudo systemctl start sensor-computer-jazzy` after provisioning.

## Calibration and later checks

Validate projection accuracy on the calibrated rig whenever the camera or LiDAR
mount changes. The initial out-of-order camera issue was traced to a LIFO driver queue and
corrected during the later [reliability review](RELIABILITY.md).
## Windows follow-up — 2026-11-25

Device 8 was tested live from the monitoring PC at `10.20.0.10/24`. The tests used read-only subscriptions.

| Check | Observed result |
| --- | --- |
| ROSBridge on `ws://10.20.0.61:9090` | Three valid `/merged_colored_cloud` messages with advancing timestamps, plus `/livox/imu`, `/temperature`, and `/image_enhancement/status`. |
| Camera and raw LiDAR | `/camera/image_raw` delivered a 484×366 `bayer_rggb8` frame. `/livox/lidar` delivered a 24,000-point cloud. |
| DDS interface selection | A temporary Cyclone DDS participant verified that selecting `SENSOR_IP` advertises `10.20.0.61` on this dual-address interface. |
| Windows native DDS after the profile update | Five valid colourized clouds with advancing timestamps, plus IMU, temperature, and enhancement status, using the Central app's Cyclone DDS settings on domain `0`. |
| Recovery after a power cycle | The sensor Ethernet profile activated about 12 seconds into boot. Systemd reported the live pipeline ready after about 35 seconds. |
| Deployment guards | All nine repository tests passed on Windows. |

For Windows native delivery, use the [transport check](TRANSPORTS.md#windows-native-setup)
after applying DDS settings and restarting the sensor service. The [boot audit
procedure](RELIABILITY.md#actual-reboot-verification) records recovery after a reboot.
