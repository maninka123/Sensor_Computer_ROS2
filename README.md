# Sensor Computer ROS 2

ROS 2 Jazzy on the ROCK 5C synchronises a FLIR Blackfly S camera and a Livox Avia LiDAR and publishes a colourized point cloud. The pipeline runs in Ubuntu 24.04 ARM64 containers on the Radxa Debian 12 host.

- Colourized point clouds, camera frames, LiDAR and IMU data.
- Native ROS 2 DDS and ROSBridge for the Central Monitoring Platform.
- Low-light enhancement with thermal protection and automatic pipeline recovery.

## One monitoring PC, multiple sensors

| Computer | Ethernet IP | ROSBridge endpoint |
| --- | --- | --- |
| Windows monitoring PC | `10.20.0.10/24` | Connects to each sensor |
| Device 8 sensor PC | `10.20.0.61/24` | `ws://10.20.0.61:9090` |
| Example additional sensor PC | `10.20.0.71/24` | `ws://10.20.0.71:9090` |

Keep the monitoring PC at `10.20.0.10`. Give each new sensor PC an unused address on the monitoring network and add its own endpoint to the app. Device 8 uses DDS domain `0`. Give additional ROS 2 rigs with the same topic names separate DDS domains and use the matching domain in each app profile. The `10.30.0.x` examples in the ROS 1 guide describe a separate monitoring network.

## Setup and verification

- [Setup and operation](ros2/README.md) covers the sensor host, network addresses, and startup.
- [Transport settings](ros2/TRANSPORTS.md) covers DDS, ROSBridge, and Windows checks.
- [Live Device 8 results](ros2/COMMISSIONING.md) records hardware commissioning and the Windows follow-up. Camera, LiDAR, colourized clouds and IMU passed the Windows ROSBridge check. Native DDS discovery timed out in the same follow-up.
- [Recovery tests](ros2/RELIABILITY.md), [software verification](ros2/VERIFICATION.md), and [ROS 1 migration](ros2/MIGRATION.md).

SDK binaries, logs, build outputs and runtime state are excluded. See [LICENSE](LICENSE) and the [ROS 1 repository](https://github.com/maninka123/Sensor_Computer).

## UNSW collection

Pushes to `main` update `Sensor_Computer_ROS2/` in the [UNSW AMT collection](https://github.com/unsw-edu-au/AMT-project) through GitHub Actions. See [repository sync and working folders](docs/amt-repository-sync.md).
