#!/usr/bin/env python3
"""Low overhead ROS 2 status monitor; does not subscribe to bulk clouds/images."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Float32, Float64, UInt32, UInt64


def main():
    rclpy.init()
    node = Node('sensor_computer_monitor')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    values = {}
    subs = []
    types = {'/temperature': Float32, '/image_enhancement/status': Bool,
             '/enclosure_correction/status': Bool, '/voxel_downsampling/status': Bool,
             '/voxel_leaf_size/status': Float64, '/merged_colored_cloud/matched_frames': UInt32,
             '/merged_colored_cloud/input_frames': UInt32,
             '/merged_colored_cloud/point_count': UInt32,
             '/merged_colored_cloud/dropped_batches': UInt32,
             '/image_enhancer/processed_frames': UInt64}
    for topic, typ in types.items():
        subs.append(node.create_subscription(typ, topic,
            lambda msg, *, t=topic: values.__setitem__(t, (msg.data, time.monotonic())), qos))
    def report():
        now = time.monotonic()
        print(json.dumps({topic: {'value': data, 'age_s': round(now-at, 1)}
                          for topic, (data, at) in values.items()}), flush=True)
    node.create_timer(5, report)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
