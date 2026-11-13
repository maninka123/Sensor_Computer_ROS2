# DDS and ROSBridge contract for the Central Monitoring Platform

The sensor publishes native ROS 2 Jazzy topics through Cyclone DDS and keeps
ROS 2 `rosbridge_server` available at `ws://10.20.0.61:9090` concurrently.
Windows owns transport selection, successful-reception validation, hysteresis,
deduplication and fallback. The ROCK never stops ROSBridge based on discovery
or subscriber counts. A bridge crash or failed handshake causes supervised
recovery. No ROS 1 Master, TCPROS or XMLRPC service is part of this deployment.

## Domain and device identity

This rig uses domain **0**, matching the supplied Device 8 configuration in
`deploy/device_8.env.example`. Device number 8 has no special transport path.
The sensor has one pipeline with the original absolute topic names. Every
additional ROS 2 rig using these names must use a distinct configured DDS domain
or a separately implemented and verified fully namespaced pipeline. An IP
address does not isolate DDS topics. Two rigs with these global names in the
same domain can mix acquisition/control data even inside their ROSBridge servers.

For future universal Central support, default absent `ROS_VERSION` to 1 and
leave the existing ROS 1 modes, native code, timeouts and regressions intact.
For ROS 2, choose DDS or ROSBridge from declared capabilities and actual fresh
delivery. Isolate gateway processes/contexts by device/domain and validate the
`(domain, namespace, topic)` ownership mapping. Reject ambiguous mappings; do
not silently use a device IP to identify writers on a shared DDS topic. Failure
of one gateway must not tear down others. Central application modifications
are deferred to that application's workspace; this repository supplies the
sensor interface and portable acceptance utility only.

## Topics, types and QoS

| Topic | ROS 2 type | Publisher / recommended receiver QoS |
|---|---|---|
| `/merged_colored_cloud` | `sensor_msgs/msg/PointCloud2` | Reliable, volatile, keep-last 1 |
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Reliable driver output; sensor readers accept best effort |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Reliable driver output; receiver keep-last 1–10 |
| `/imu/data` | `sensor_msgs/msg/Imu` | Complementary-filter output |
| `/temperature` | `std_msgs/msg/Float32` | Reliable, transient-local, keep-last 1 |
| `/image_enhancement/status` | `std_msgs/msg/Bool` | Reliable, transient-local, keep-last 1 |
| `/enclosure_correction/status` | `std_msgs/msg/Bool` | Reliable, transient-local, keep-last 1 |
| `/voxel_downsampling/status` | `std_msgs/msg/Bool` | Reliable, transient-local, keep-last 1 |
| `/voxel_leaf_size/status` | `std_msgs/msg/Float64` | Reliable, transient-local, keep-last 1 |
| `/merged_colored_cloud/{matched_frames,input_frames,point_count,dropped_batches}` | `std_msgs/msg/UInt32` | Reliable, transient-local, keep-last 1 |
| `/image_enhancement` | `std_msgs/msg/Bool` | Reliable, volatile control |
| `/enclosure_correction` | `std_msgs/msg/Bool` | Reliable, volatile control |
| `/voxel_downsampling` | `std_msgs/msg/Bool` | Reliable, volatile control |
| `/voxel_leaf_size` | `std_msgs/msg/Float64` | Reliable, volatile control, metres |

Read effective `/status` values after commands; thermal protection can keep
enhancement off despite a user request. Do not latch control commands. Avoid
automatically replaying a command whose delivery is uncertain. Use one selected
control path at a time and preserve the existing app's latest-request policy.

The final cloud retains 36-byte points and named fields `x`, `y`, `z`, `rgb`,
`source_frame_index`, `source_frame_stamp_sec`, `source_frame_stamp_nsec`,
`image_stamp_sec`, and `image_stamp_nsec`. Read offsets/types from `fields`;
do not assume every incoming cloud has a fixed layout. Packed RGB remains a
float field containing uint32 colour bits. Validate `row_step * height` against
the data length and validate all required provenance fields before selection.

ROS 2 headers have `stamp.sec` and `stamp.nanosec` and no ROS 1 `seq`. The
per-point `_sec`/`_nsec` names above are retained intentionally. JSON ROSBridge
encodes `uint8[] data` as base64 in the tested version. Decode to bytes without
changing the message's timestamps, endian flag, row stride or field definitions.

## Windows native setup

Use a supported Jazzy installation and a compatible RMW, the same domain, and
an adapter on `10.20.0.0/24`. Cyclone DDS is the ROCK's RMW. A Jazzy Fast DDS
client successfully received its clouds, IMU and transient-local status in a
real same-host Linux interoperability test. That establishes middleware-level
interop; Windows binaries, firewall and the physical link still need testing.

`deploy/fastdds-windows.example.xml` is a private-interface/unicast-peer example
for Central `10.20.0.10`, sensor `10.20.0.61` and domain 0. Review it against the
actual Windows adapter and installed RMW version. Its discovery ports correspond
to domain 0 and the first 24 participants. The tested loopback counterpart is
`deploy/fastdds-test.xml` (domain 73); never use that test profile on Windows.
The peer/interface mechanisms follow the upstream
[Fast DDS discovery documentation](https://fast-dds.docs.eprosima.com/en/stable/fastdds/discovery/simple.html)
and [interface whitelist documentation](https://fast-dds.docs.eprosima.com/en/stable/fastdds/transport/whitelist.html).

From a Jazzy-enabled Windows PowerShell, with the example XML and client script copied locally:

```powershell
$env:ROS_DOMAIN_ID = "0"
$env:RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"
$env:FASTRTPS_DEFAULT_PROFILES_FILE = (Resolve-Path .\fastdds-windows.example.xml)
python .\central_transport_check.py --transport dds
```

ROSBridge-only validation needs Python and `websocket-client`, without a native
ROS installation:

```powershell
python .\central_transport_check.py --transport websocket --url ws://10.20.0.61:9090 --check-controls
```

`--check-controls` briefly changes voxel size, waits for feedback, then restores
the original value. `--transport both` checks both paths. The exact utility was
tested as a separate Linux DDS client and JSON WebSocket client against the
ported pipeline, including control acknowledgement/restoration. It is a test
utility, not the Central application's transport manager.

Permit DDS UDP traffic between the two private sensor-network addresses in
Windows Firewall and the ROSBridge TCP connection to port 9090. Discovery and
data ports both matter; a successful ping or discovered node does not establish
delivery. Restrict rules to the sensor LAN/address. No broad host firewall
disablement or ROSBridge listener on the campus/internet address was introduced.

## Selection and failover responsibilities

Start a DDS probe while a bridge fallback remains available. Prefer DDS only
after receiving several structurally valid, correctly attributed clouds with
advancing timestamps and acceptable freshness/gaps. Use a stable interval
before promoting DDS and a separate loss threshold before falling back, both
scaled to the actual roughly one-second fusion output interval. Discovery,
participant presence and matched endpoints alone are insufficient.

Keep one active ingest path per device. Increment a connection generation on
handoff and discard callbacks from previous generations; deduplicate overlapping
messages using device identity plus source/image stamps rather than a ROS 1
sequence number. Keep status/control subscriptions independent of bulk cloud
selection when useful, without replaying stale controls. Report the chosen
transport from actual reception state, and keep trying a failed DDS path with
bounded backoff. The bridge continues listening on the ROCK throughout.

## Measurement and remaining validation

`tools/transport_benchmark.py` measures native DDS and JSON WebSocket delivery of
synthetic 100,000-point / 3.6 MB messages on an isolated test topic. The container
recovery test runs it with 1400-byte RTPS messages and records latency, bytes and
delivery counts. It does not modify `/merged_colored_cloud`. See
[actual results](VERIFICATION.md).

The same-host measurements include software serialization/client scheduling,
not physical Ethernet or Windows latency. JSON's base64 expansion and observed
large-frame delay make native DDS preferable for large cloud streams in this
test. Real links still need per-transport delivered rates, byte rates, sample
age, jitter, disconnect/reconnect and overload testing. Reliable keep-last-one
limits history; it is not a lossless recording guarantee. Use synchronized clocks
before interpreting one-way age between machines. Compression/CBOR may be
evaluated only when the actual Central client supports and validates it.
