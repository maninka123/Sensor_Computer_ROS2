"""Real DDS regression for Avia uptime conversion, SI units and clock resets."""
import os
import signal
import subprocess
import time
import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu


class LivoxImuTest(unittest.TestCase):
    def test_shared_clock_units_and_sensor_restart(self):
        rclpy.init()
        node = Node('imu_normalization_fixture')
        received = []
        lidar = node.create_publisher(PointCloud2, '/test/lidar', 10)
        imu = node.create_publisher(Imu, '/livox/imu_raw', 10)
        sub = node.create_subscription(Imu, '/livox/imu', received.append, 10)
        process = subprocess.Popen([
            'ros2', 'run', 'node_pc', 'lidar_timestamp_shift_node', '--ros-args',
            '-p', 'auto_timestamp_offset:=true', '-p', 'normalize_livox_imu:=true',
            '-p', 'input_topic:=/test/lidar', '-p', 'output_topic:=/test/shifted',
        ], stdout=subprocess.DEVNULL, start_new_session=True)
        def spin(seconds):
            end = time.monotonic()+seconds
            while time.monotonic()<end:
                rclpy.spin_once(node, timeout_sec=.02)
        def message(second):
            result = Imu()
            result.header.stamp.sec = second
            result.header.stamp.nanosec = 80000000
            result.linear_acceleration.z = 1.0
            result.angular_velocity.x = .25
            return result
        try:
            deadline = time.monotonic()+10
            while (not lidar.get_subscription_count() or not imu.get_subscription_count()) and time.monotonic()<deadline:
                spin(.1)
            self.assertGreater(lidar.get_subscription_count(), 0)
            imu.publish(message(100))
            spin(.3)
            self.assertEqual(received, [], 'Must wait for a LiDAR clock anchor')
            for device_time in [100, 1]:
                before = len(received)
                cloud = PointCloud2()
                cloud.header.stamp.sec = device_time
                lidar.publish(cloud)
                spin(.2)
                imu.publish(message(device_time))
                deadline = time.monotonic()+3
                while len(received)==before and time.monotonic()<deadline:
                    spin(.05)
                self.assertGreater(len(received), before)
                actual = received[-1]
                self.assertLess(abs(time.time()-actual.header.stamp.sec-actual.header.stamp.nanosec/1e9), 1)
                self.assertAlmostEqual(actual.linear_acceleration.z, 9.80665)
                self.assertAlmostEqual(actual.angular_velocity.x, .25)
                self.assertEqual(actual.orientation_covariance[0], -1)
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
