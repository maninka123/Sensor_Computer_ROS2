# ROCK 5C environment and migration decision

Survey: 2026-11-13 UTC. Upstream Sensor_Computer commit: 2a70338018ee8826b4b0b1ba6649f2ecf149207c.
Development branch: deploy/rock5c-debian12-jazzy.

Host: Radxa ROCK 5C, aarch64, Debian 12 bookworm, vendor kernel 6.1.43-15-rk2312.
8 GiB RAM (about 4.7 GiB available initially), 4 GiB swap, 211 GiB free on 229 GiB eMMC root filesystem.
Desktop is KDE Plasma (confirmed from the active session), launched by GDM; SDDM is not in use. Built-in Ethernet is end1 (initially no carrier).
USB Ethernet enx207bd2b1fe36 supplies DHCP internet, default route and DNS; preserve it.
NetworkManager manages networking. No preexisting ROS, Docker, compiler or CMake was found.
Camera and LiDAR were not visible on a connected sensor LAN at survey time.

## Architecture decision

Use the official ARM64 ros:jazzy-ros-base-noble image (Ubuntu 24.04), host networking,
Cyclone DDS and native ament/colcon packages. Keep Debian, bootloader, partitioning,
Radxa drivers and kernel intact. Keep the original ROS 1 source and deployment for reference;
new ROS 2 code lives in ros2/. No ROS 1 master, ROS_IP or TCPROS switching belongs in ROS 2 startup.
The existing four C++ processing algorithms and numerical calibration should be ported with
mechanical middleware changes, then verified with synthetic inputs before real sensors.
ROSBridge remains available on the sensor address; native DDS does not disable it because
DDS graph discovery alone cannot prove an actual remote subscription/data delivery.

## Compatibility matrix

| Component | Finding | Action / verification needed |
|---|---|---|
| Debian 12 ARM64 host | Docker packaged by Debian; vendor kernel includes namespaces, cgroups, seccomp, overlay support | Install Debian docker.io, verify actual ARM64 container startup |
| Jazzy / Noble | Official ROS image supports arm64v8 | Pull and execute image; record digest |
| Original application | Catkin, roscpp/rospy, ROS 1 launch and master assumptions | Native ament/rclcpp/rclpy migration; retain algorithms and message layouts |
| Livox Avia | First generation original Livox SDK device; official Livox-SDK/livox_ros2_driver supports Avia | Pin driver and SDK; adapt older ROS 2 APIs for Jazzy; test real packets |
| Livox SDK2 / driver2 | Supports newer devices, including Avia2; Avia2 is not original Avia | Do not substitute it for original Avia |
| FLIR Blackfly S | ROS 2 driver available from ros-drivers/flir_camera_driver | Pin source; preserve 4x4 binning, 35 FPS and GigE settings |
| Spinnaker | Original Noetic requires 4.2.0.46/Focal; user supplied 4.4.0.246/Noble ARM64 | Installed 4.4.0.246 inside container; dynamic dependencies, SDK loading and ROS 2 driver build passed; acquisition pending |
| Inference | Python 3.12 ARM64 wheels: Torch 2.6 CPU, ONNX Runtime 1.20.1, NumPy 1.26.4 | Included TorchScript model and ROS enhancement/thermal tests passed |
| ROSBridge | ROS 2 compatible rosbridge_server is available | Bind 10.20.0.61:9090; test controls and PointCloud2 |
| Remote client | ROS 1 TCPROS client cannot directly connect to DDS | Use Jazzy/domain-compatible DDS or existing WebSocket protocol with ROS 2 type names |
| Calibration | Existing camera matrix/extrinsics apply to the original physical rig | Preserve values; actual mounting/serial must be verified before accepting geometry |

Sources checked: https://hub.docker.com/_/ros ; https://github.com/Livox-SDK/livox_ros2_driver ;
https://github.com/Livox-SDK/livox_ros_driver2 ; https://github.com/ros-drivers/flir_camera_driver ;
https://www.teledynevisionsolutions.com/products/spinnaker-sdk/ ; https://docs.docker.com/engine/network/drivers/host/ .

## Controlled implementation stages

1. Back up relevant host configuration; install Docker and diagnostic tools only.
2. Pull/test Jazzy ARM64; build processing and Livox components separately from proprietary FLIR SDK.
3. Port C++ nodes, image enhancement, launch, parameters, QoS, diagnostics and bounded stream health checks.
4. Test actual DDS delivery, synthetic raw camera/LiDAR -> fused cloud, controls, thermal fallback and ROSBridge.
5. Prepare rollback-capable NetworkManager sensor profile (10.20.0.61/24 and 192.168.1.50/24 on end1).
6. Obtain and verify compatible Spinnaker archive; build/test FLIR acquisition and real Avia/IMU streams.
7. Install/enable permanent services and desktop autologin only after explicit approval and live acceptance checks.

## Power and clock evidence

Several boots are recorded within minutes on 2026-11-13. Earlier boots begin at a 2024 clock value,
then jump to current time; wtmp therefore reports misleading crash durations. No explicit brownout,
undervoltage, kernel panic or OOM evidence was found in the current kernel journal. Regulator messages
report configured supply values, not a measured power supply. Software logs cannot certify PSU quality;
power-loss causes remain unverified. Thermal zone0 is soc-thermal; initial temperature about 63 C.
Confirm stable supply under real combined sensor/CPU load. Boot without valid wall time must be handled
before accepting synchronized data; investigate RTC/NTP availability in offline operation.

## Host changes already made

Debian docker.io, ripgrep, ethtool, iputils-arping and their package dependencies installed.
Docker's package scripts initially enabled Docker/containerd; those boot enables were removed pending
approval. Docker is running for development. User rock added to docker group for build access.
No sensor address, desktop login, application startup, kernel or firmware change has been applied.

## Implementation outcome

Jazzy image executed; complete six-package source build passed with supplied Noble SDK.
Nine real-DDS software integration cases and the container recovery checks passed.
A separate Fast DDS client received the Cyclone pipeline data. Native DDS and ROSBridge
remain available together. The production domain is 0, matching the user's latest device
configuration. Additional rigs with identical topics require distinct domains or a verified
namespace migration; IP addresses alone are not DDS isolation.

Automatic sleep/hibernate and Plasma lock policies were disabled at the user's explicit
request. The application unit is installed but disabled pending activation approval.
Network/static-profile and desktop-autologin changes remain staged for that decision.
See ../ros2/VERIFICATION.md for actual outcomes and hardware-dependent gaps.
