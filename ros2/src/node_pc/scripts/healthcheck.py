#!/usr/bin/env python3
"""Require actual fresh sensor and output messages, not just DDS node discovery."""
import argparse
import json
import os
import time
import shutil
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, Imu
from stream_sample import FreshnessTracker


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--startup-timeout', type=float, default=120)
    parser.add_argument('--max-age', type=float, default=20)
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--no-imu', action='store_true')
    parser.add_argument('--bridge-url', default='')
    parser.add_argument('--bridge-timeout', type=float, default=5)
    parser.add_argument('--bridge-check-interval', type=float, default=15)
    parser.add_argument('--bridge-retries', type=int, default=3)
    parser.add_argument('--storage-path', default='/tmp')
    parser.add_argument('--min-free-mb', type=float, default=2048)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node('pipeline_healthcheck')
    tracker = FreshnessTracker()
    seen = tracker.seen
    topics = {'/camera/image_raw': Image, '/livox/lidar': PointCloud2,
              '/merged_colored_cloud': PointCloud2}
    if not args.no_imu:
        topics['/livox/imu'] = Imu
    subscriptions = []
    for topic, message in topics.items():
        subscriptions.append(node.create_subscription(
            message, topic, lambda data, *, t=topic, geometry={Image:'image',PointCloud2:'cloud',Imu:'imu'}[message]: tracker.observe(t,data,geometry),
            qos_profile_sensor_data, raw=True))
    started = time.monotonic()
    healthy_once = False
    next_report = started
    next_bridge_check = started
    bridge_ok = not bool(args.bridge_url)
    bridge_failures = 0
    result = 1
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            now = time.monotonic()
            missing = [topic for topic in topics if now-seen.get(topic, -float('inf')) > args.max_age]
            if shutil.disk_usage(args.storage_path).free < args.min_free_mb*1024*1024:
                print('FAIL: storage has less than the configured free-space reserve', flush=True)
                break
            if args.bridge_url and now >= next_bridge_check:
                import websocket
                try:
                    connection = websocket.create_connection(args.bridge_url, timeout=args.bridge_timeout)
                    connection.close()
                    bridge_ok = True
                    bridge_failures = 0
                except Exception:
                    bridge_ok = False
                    bridge_failures += 1
                next_bridge_check = now+args.bridge_check_interval
            if not bridge_ok:
                missing.append('ROSBridge WebSocket')
            if not missing:
                if not healthy_once:
                    print('PASS: fresh camera, LiDAR, fused cloud and required IMU streams', flush=True)
                healthy_once = True
                if args.once:
                    result = 0
                    break
            fatal_missing = [item for item in missing if item != 'ROSBridge WebSocket']
            if bridge_failures >= args.bridge_retries:
                fatal_missing.append('ROSBridge WebSocket')
            if (healthy_once and fatal_missing) or (not healthy_once and now-started > args.startup_timeout):
                print('FAIL: missing or stalled streams: '+', '.join(missing), flush=True)
                break
            if now >= next_report:
                print(json.dumps({'healthy': not missing, 'missing': missing,
                                  'bridge_failures': bridge_failures}), flush=True)
                next_report = now+15
    except (KeyboardInterrupt, ExternalShutdownException):
        result = 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(result)


if __name__ == '__main__':
    main()
