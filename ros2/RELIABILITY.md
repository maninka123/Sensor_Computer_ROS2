# Reliability review — 24 November 2026

This record distinguishes faults actually injected on the ROCK 5C from physical
and Windows-side acceptance that still needs observation. It is not a guarantee
that every possible hardware or software failure has been eliminated.

## Corrections

- Fixed the pinned FLIR driver's LIFO image deque consumption to FIFO. A brief
  publishing backlog previously emitted older timestamps after newer frames.
  Fusion ignores isolated late frames up to 0.5 s; larger backward clock jumps
  still clear old history. The pinned driver preparation reproduces the change.
- Health checks now validate image/cloud/IMU payload length and cloud field
  bounds in raw Jazzy CDR, plus finite IMU values. A header with no data no longer
  counts as a working stream. A repeating cycle of the last 32 cached timestamps
  cannot keep the watchdog green.
- Filter/colorizer reject truncated indexed clouds, incorrect field types,
  out-of-bounds fields, unsupported endian/layout and invalid source timestamps.
  Projection checks finite pixel coordinates before integer conversion.
- Voxel size controls accept 0.0001–10 m; NaN, infinity, zero, negative and extreme
  underflow/overflow values preserve the previous setting and publish feedback.
- Bridge watchdog probes allow 5 s for a handshake and require three consecutive
  failures before restarting an already healthy pipeline. An isolated failed probe
  is reported without disrupting valid sensor acquisition. A real regression
  verifies transient-failure tolerance and sustained-failure detection. This
  fixes an observed false restart during a large JSON cloud test.
- Startup waits for Ethernet carrier and both static addresses. Core dumps are
  capped at 64 MiB; Docker logs, journal storage, RAM and process counts are bounded.
- Network setup rollback now restores an existing static profile by UUID on a
  failed reactivation. It only deletes a profile if that invocation created it;
  it no longer assumes a separate DHCP profile exists. The live static profile
  was not deliberately broken to test this installation-only failure path.
- The boot audit runs 45 s after boot and retries every 60 s. It records actual
  changing sensor payloads, SSH and ROSBridge, saved/active addresses, Ethernet
  carrier, enabled/running services, one managed container and software acceptance.
  It writes JSON atomically with fsync and retains one record per boot ID.

## Completed checks

| Check | Result |
|---|---|
| Deployment guard tests | 9 passed, including frozen/replayed stamps and strict empty/truncated image rejection. |
| Real DDS IMU normalization regression | Passed: first clock anchor, acceleration units and sensor clock reset. |
| Real Jazzy pipeline integration | 11 passed, including malformed cloud injection, late images, invalid controls, temperature failure/thermal fallback, backward clocks, native DDS/Fast DDS, ROSBridge controls and essential-process exit. |
| Real Jazzy serialization | Actual `serialize_message` images, clouds and IMU accepted; truncated versions rejected. |
| Camera UDP interruption | 28 s deliberately blocked; 16,237 packets / 22,324,156 bytes dropped. Watchdog restarted the pipeline and independent fresh camera/LiDAR/IMU/fused checks passed after traffic restoration. |
| Avia UDP interruption | 28 s deliberately blocked; 15,893 packets / 20,484,586 bytes dropped. Watchdog restarted the pipeline and independent live checks passed afterward. |
| Normal Docker daemon restart | Passed; a new managed container started and fresh streams returned. |
| Forced Docker daemon crash | SIGKILL to the daemon; systemd restarted Docker, the pipeline recovered and fresh streams passed. Recovery probe completed in 38.67 s. One managed container remained. |
| Final live transports | Three valid live clouds through each transport plus IMU/temperature/status; ROSBridge command acknowledgement and restoration passed after watchdog hardening. |
| Duplicate-node review | One managed container and eleven unique pipeline node names; no duplicate essential nodes in the final graph. |
| Isolated bridge crash | Production-style test container restarted and restored the endpoint. A deliberate stop stayed stopped. |
| Bulk transport test | Both DDS and ROSBridge delivered all 10 synthetic 3.6 MB clouds. This is a same-host measurement, not Windows Ethernet throughput certification. |
| Host review | SSH remains enabled; sleep/hibernate targets masked; no ROS 1 service units found. CPU temperature about 59°C; no kernel OOM, storage-I/O, critical thermal or undervoltage errors found in the reviewed current-day journal. |

Additional live checks passed: killing the Avia driver led to fresh-stream
recovery (probe completed in 46.30 s). Blocking the Windows peer's DDS and
ROSBridge traffic for 20 s left the sensor pipeline healthy without a restart.
The corrected camera published 2,098 observed images in 60 s with zero backward
or duplicate timestamps. Evidence is local under
`.local-deploy/reliability-2026-11-24/`. Targeted traffic tests use temporary chains with
an independent systemd rollback timer. They do not block TCP port 22 or change
addresses, gateways or the USB installation connection.

## Actual reboot verification

Run these only after software acceptance and live health pass:

```bash
sudo python3 /usr/local/lib/sensor-computer/boot_audit.py --prepare-reboot
sudo systemctl reboot
```

The first command refuses to save a baseline unless the pipeline is healthy. It
saves the current boot ID, configuration hashes and running image ID. It does
not itself reboot. After a real reboot the audit compares that baseline and
sets `reboot_test.passed` only when all of the following hold: boot ID changed,
configuration and image are unchanged, boot services are enabled/active, one
managed container exists, the expected addresses/carrier and SSH are present,
and real changing camera, LiDAR, IMU and fused payloads plus ROSBridge are healthy.
A late sensor/network startup is retried on the next audit interval.

```bash
sudo cat /var/lib/sensor-computer/boot-audit.json
sudo journalctl -u sensor-computer-boot-audit.service --no-pager
sudo journalctl --namespace=sensor-computer -u sensor-computer-jazzy.service -f
systemctl is-enabled docker ssh sensor-computer-jazzy
sudo systemctl disable --now sensor-computer-jazzy  # maintenance
```

The local post-boot JSON is the record for a physical reboot. It also records
clock synchronization. Offline absolute time still needs a valid clock source;
the pipeline starts independently of the installation internet connection.

## Limits requiring physical or remote observation

A software traffic block exercises network loss and recovery; it does not test
PoE power stability, a failing switch, unplugging a cable, electrical brownouts
or corrupted hardware. A software reboot does not certify sudden power-loss
behaviour. No disk-filling, forced host OOM or hard power cut was performed.
Long-duration running and camera/LiDAR calibration accuracy remain to be tested.
The default pipeline does not record local rosbag files, so recording durability
has not been certified. See the [Windows follow-up](COMMISSIONING.md#windows-follow-up--2026-11-25)
for read-only ROSBridge reception and the native DDS discovery result.
The server continues to expose DDS and
ROSBridge together; it does not decide transport from subscriber counts.
