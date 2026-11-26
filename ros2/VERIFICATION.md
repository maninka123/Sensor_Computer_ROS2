# ROCK 5C verification record

See the [live commissioning and Windows results](COMMISSIONING.md) for the
current Device 8 record. The November 13, 2026 results below used synthetic inputs
while hardware and the Windows monitoring computer were unavailable.

## Completed checks

| Check | Actual result |
|---|---|
| Host survey | Debian 12 ARM64, ROCK 5C, vendor 6.1.43-15-rk2312 kernel, 8 GiB RAM, eMMC; NetworkManager; KDE Plasma under GDM. |
| Docker / official Jazzy image | ARM64 Ubuntu Noble container executed successfully. Base digest `sha256:00b8d47a6e6384a2834e3d01fa87ae767876543baa574f2d64a75c04ea33c698`. |
| Dependencies | OpenCV 4.6.0, PCL 1.14, GCC 13, ROS 2 Jazzy, Python 3.12; both sensor executables present with no unresolved shared libraries. No `/opt/ros/noetic` in the image. |
| Spinnaker | Supplied Noble ARM64 4.4.0.246 package installed in container; headers/library found; `ldd` passed. SDK probe actually loaded the library, reported version 4.4.0.246 and **zero cameras**, consistent with absent hardware. |
| Inference | Torch 2.6.0+cpu, NumPy 1.26.4, ONNX Runtime 1.20.1, cv_bridge import; included TorchScript model executed a 1x3x366x484 input. |
| Full colcon build | All six packages built: flir_camera_msgs, livox_interfaces, livox_sdk_vendor, node_pc, livox_ros2_driver, spinnaker_camera_driver. |
| Numerical configuration | Original camera intrinsics, distortion, extrinsics and filtering/synchronization values retained; only matrix representation changed to flat ROS 2 arrays. |
| Integration tests | **9 passed**, final recorded run 36.788 seconds. Real Jazzy C++/Python nodes and DDS were used with synthetic camera/LiDAR/IMU inputs. |
| Original repository tests | Original 3 ROS 1 transport-supervisor unit tests passed before and after changes. Original ROS 1 implementation files were preserved. These are not Central application regression tests. |
| Portable Central client | Separate Cyclone DDS client received 3 valid clouds plus IMU, temperature and effective enhancement state. JSON ROSBridge client received the same categories and verified voxel-size command/feedback/restoration. |
| Fast DDS interop | Separate Jazzy `rmw_fastrtps_cpp` client received 3 valid clouds, IMU, temperature and transient-local status from the Cyclone DDS pipeline using private loopback discovery. |
| Container lifecycle | Unprivileged, capability-free, read-only production-style container started. Killing ROSBridge caused complete container restart and a working replacement endpoint. Deliberate stop remained stopped. RAM/process limits inspected. |
| Storage guard | A deliberately impossible free-space requirement caused healthcheck failure as designed. No disk-filling operation was performed. |
| Sleep prevention | All five system sleep/hibernate targets report masked. Plasma AC/battery/low-battery idle suspend disabled, DPMS timeout disabled, automatic lock disabled. Current session was not restarted. |
| Service validation | Installed root-owned launcher, runtime config and disabled unit passed systemd verification. A pre-existing world-writable Radxa camera service generated a host warning unrelated to this unit; its vendor file was not changed. |
| Syntax / repository hygiene | Python compilation, shell syntax and `git diff --check` passed. Vendor archives, builds, logs and local state are ignored. |

The nine integration cases cover native fusion and point provenance, control
changes/invalid leaf-size rejection, enhancement processing, 85 C thermal
fallback and 70 C recovery, temperature-file failure, enclosure correction,
ROSBridge streaming/control and the portable client, Fast DDS interoperability,
fresh-stream success/missing-stream failure, backward timestamps and recovery,
and an essential-node crash terminating the launch. Closely related checks are
grouped inside the nine test methods.

## Large-message delivery check

A temporary read-only test container on loopback domain 74 published ten
100,000-point messages, 36 bytes/point (3.6 MB payload each), using reliable
keep-last-one QoS and 1400-byte RTPS packets. A native DDS subscriber and a JSON
ROSBridge client ran concurrently. This test is separate from the production
output topic and uses synthetic data.

| Measurement | Recorded value |
|---|---:|
| Native DDS received | 10 / 10 |
| ROSBridge JSON received | 10 / 10 |
| Total received WebSocket JSON bytes | 48,007,908 |
| Complete test elapsed time | 18.352 s |
| JSON wire bytes / complete elapsed time | approximately 20.9 Mbit/s |

The same-host loopback check verified delivery, message layout, byte length
and ROS 2 header serialization for both transports. It used synthetic clouds.
Use native DDS for large live clouds and bounded, latest-frame subscriptions
when using ROSBridge. The [Windows follow-up](COMMISSIONING.md#windows-follow-up--2026-11-25)
records live reception on both transports.

## Evidence and reproducible commands

Local evidence is under `../.local-deploy/`, including:

```
jazzy-pull.log                 jazzy-full-build.log
dependency-verification.log    inference-sdk-check.log
sdk-discovery.log              integration-tests.log
integration-launch.log         central-client-local-check.log
fastdds-interoperability.log   container-recovery-tests.log
container-recovery-runtime.log transport-benchmark.json
storage-guard-test.log         upstream-tests.log
upstream-tests-after.log       service-install.log
```

Re-run `./ros2/tools/verify.sh` and
`sg docker -c 'python3 ros2/tools/container_recovery_test.py'` when source,
dependencies or transport configuration changes. Temporary clients use
loopback ports 19090/19091 and domains 73/74, not the production endpoint.

## Hardware checks outside the original software test

Actual Avia LiDAR and IMU acquisition; physical camera discovery and acquisition;
camera's persistent IP/settings; actual 484x366 / 35 FPS output; live timestamp
accuracy; colour projection/calibration; actual LiDAR/IMU/output rates; real
Windows DDS discovery, message delivery and WebSocket controls; physical-link
latency/bandwidth; link and sensor unplug/reconnect; power loss; repeated reboots;
desktop logout independence in an actual logout; long-duration operation and
resource/thermal behavior with hardware; and successful boot-time live health.

The USB internet adapter is installation-only; production startup neither waits
for it nor downloads dependencies. Existing systemd-timesyncd persists a recent
clock timestamp, but offline time while powered off remains unknowable without
a clock source. The sensor Ethernet initially has no carrier. The persistent NetworkManager profile and desktop autologin have been applied.
The user subsequently authorized boot startup; readiness is gated on real changing
sensor samples, and a boot audit records sensor readiness separately.

The Central application itself was not changed. Universal mixed ROS 1/ROS 2
connection-manager work is deferred to its own workspace, as clarified by the
user. Existing ROS 1 application regressions and multi-device platform failover
are therefore not claimed here.

## Additional corner-case review

Seven deployment guard tests cover malformed/empty CDR samples, endian handling,
frozen stamps, backward clocks, stale/missing acceptance, image changes and source
changes during testing. Fresh-clone state creation and SDK version/architecture
preflight are included. Successful acceptance is bound to the exact source, built
artifacts and image, and old pass markers are removed before rerunning checks.
Systemd startup waits for actual data, owns retries, and reattaches only its own
labelled containers. Stop hooks never terminate an unowned manual container.
The [Device 8 commissioning follow-up](COMMISSIONING.md#windows-follow-up--2026-11-25)
records automatic startup. The boot audit writes its results to
`/var/lib/sensor-computer`.
