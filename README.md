# Sensor Computer ROS 2 — ROCK 5C

ROS 2 Jazzy / Ubuntu 24.04 ARM64 in Docker on Radxa Debian 12. Livox Avia, FLIR Blackfly S, timestamp-aware fusion, thermal protection, DDS and always-available ROSBridge.

[Setup and operation](ros2/README.md) · [Verification results and limitations](ros2/VERIFICATION.md) · [DDS and Windows contract](ros2/TRANSPORTS.md) · [Migration](ros2/MIGRATION.md)

Sensor addresses: 10.20.0.61/24 and 192.168.1.50/24 on end1. ROSBridge: ws://10.20.0.61:9090. DDS domain 0 for this rig. Additional rigs need isolated DDS domains or verified namespaces.

Systemd starts independently of desktop login, internet and the Windows client. Readiness requires actual changing sensor/output samples. Software tests pass with synthetic data; physical sensors, Windows, reboot and power-cycle commissioning remain pending.

SDK binaries, logs, build outputs and runtime state are excluded. The original ROS 1 repository remains unchanged at https://github.com/maninka123/Sensor_Computer. See LICENSE.

## UNSW collection

Pushes to personal `main` automatically update `Sensor_Computer_ROS2/` in the [UNSW AMT collection](https://github.com/unsw-edu-au/AMT-project). See [repository sync and existing working folders](docs/amt-repository-sync.md).
