# ROS 1 source to ROS 2 Jazzy migration

The original `src/`, `run_pipeline.sh` and ROS 1 deployment are preserved in
the upstream Sensor_Computer repository and original machine checkout. New sources live under `ros2/src/node_pc`, and official
ROS 2 driver sources are pinned under the ignored `ros2/vendor` directory.
The Jazzy boot unit does not install/start the legacy transport supervisor.

| Area | Implemented change |
|---|---|
| Host compatibility | Run Ubuntu Noble ARM64 userspace in Docker on the original Debian/Radxa kernel. No host ROS apt repository, kernel, firmware or partition changes. |
| Build | Separate ament/colcon workspace, C++17, Noble OpenCV 4.6/PCL 1.14 and explicit ROS 2 dependencies. |
| C++ nodes | Port the timestamp shifter, indexed merger, per-source-frame filter and colourizer to `rclcpp::Node`, native ROS 2 messages, subscriptions, publishers and timers. Keep numerical filtering, projection, one-to-one matching and provenance layouts. |
| Time | Use ROS time explicitly for message stamps and a steady clock for batch deadlines. Convert ROS 1 `nsec` access to ROS 2 `nanosec`. Keep the 32.43 ms capture correction and live clock-offset estimation. Hardware-free fixtures use already aligned ROS timestamps, not the old recorded-bag epoch offset. |
| Parameters | Native per-node `ros__parameters`; flatten the camera matrix for ROS 2 parameter arrays. Preserve numerical intrinsics/extrinsics. Keep the existing `image_enchantment` spelling/type for compatibility. Remove ROS 1 `~` parameter prefixes. |
| QoS | Reliable, volatile, depth-one final cloud; best-effort-compatible sensor subscriptions; reliable transient-local status/temperature; reliable volatile control commands. Select transient-local subscriptions only for effective state topics. |
| Python | Port enhancement to `rclpy`; retain the latest-frame worker, CPU affinity, generation checks, max FPS, capture stamps, thermal hysteresis and raw fallback on temperature-read failure. Use Python 3.12 ARM64 inference wheels and preserve the TorchScript/ONNX assets. |
| Livox | Use official `Livox-SDK/livox_ros2_driver` at `1565b976ab1905f911cd2a757fc32433422b701c`, with original SDK revision `9306596a2bf15c1343bc023b497465ed0a32909d`. Avia is not Avia2 and does not use SDK2. |
| Legacy SDK compiler fixes | Add missing `<memory>` to `thread_base.h`; keep the SDK's warnings-as-errors policy except the old bundled fmt `char8_t` C++20-compatibility warning under GCC 13. No packet protocol or numeric SDK algorithms changed. |
| FLIR | Official ROS 2 driver at `986172dbd8dab9ac5a365d0fcbd785508803e960` plus supplied Noble ARM64 Spinnaker 4.4.0.246. Disable its automatic SDK fallback download. Add camera-node mappings for the preserved transport/brightness settings; live node availability still requires hardware verification. |
| Camera configuration | Use the commissioned serial 25120439, 4x4 binning, 35 FPS, 1400-byte packets, delay 1000, 64 host buffers, continuous exposure/gain and gamma 0.75. Unsupported sharpening, saturation and packet-resend parameter overrides are omitted for BFS-PGE-27S5C. Use receipt/system timestamps as in the original driver; do not enable unvalidated PTP. |
| IMU/TF | Retain `/livox/imu`, complementary filtering and the map-to-livox static TF using native ROS 2 nodes. |
| Communication | DDS and ROS 2 ROSBridge are always available together. No ROS 1 Master, ROS_IP, ROS_MASTER_URI, XMLRPC or TCPROS connection path. Central chooses transport after validated data receipt; discovery does not disable ROSBridge. |
| Monitoring | Replace ROS 1 CLI/XMLRPC monitoring with low-volume ROS 2 status subscriptions and actual fresh-stream checks. |
| Boot/recovery | Systemd-managed unprivileged container; local network wait, sensor/bridge health, bounded retry, essential-node exit handling, clean signals and duplicate-container protection. Restore SIGINT before executing Bash background children. |
| Logs/resources | Isolated capped journal, capped Docker logs, ephemeral bounded ROS logs, memory/process limits and storage reserve. |

The port adds no custom remote message definitions: output remains standard
`sensor_msgs/msg/PointCloud2` with the same named per-point provenance fields.
The vendor's `livox_interfaces` package is internal to the driver. Original ROS 1
message files and native ROS 1 communication code have not been rewritten.

Original ROS 1 bags need conversion to ROS 2 bag format and a deliberate clock
policy before playback. The ROS 1 `run_rosbags.sh` and `--clock` workflow must not
be used as if they were ROS 2 compatible. Real hardware remains the acceptance
gate for calibration, timestamp accuracy, sustained throughput and recovery.

The November 24 Avia uses broadcast code `3JEDN920015M711`. Hardware launch
normalizes its IMU uptime and g units before publishing `/livox/imu`; the raw
source is `/livox/imu_raw`. See [COMMISSIONING.md](COMMISSIONING.md).
