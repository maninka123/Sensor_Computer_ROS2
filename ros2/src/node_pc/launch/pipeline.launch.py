"""Native DDS pipeline. hardware:=false accepts externally supplied raw topics."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def bringup(context):
    share = Path(get_package_share_directory('node_pc'))
    params = str(share / 'config/pipeline.yaml')
    hardware = LaunchConfiguration('hardware').perform(context).lower() == 'true'
    bridge = LaunchConfiguration('rosbridge').perform(context).lower() == 'true'
    temperature_path = LaunchConfiguration('temperature_path').perform(context)
    nodes = []
    for executable, name in [
        ('lidar_timestamp_shift_node', 'lidar_timestamp_shift'),
        ('pointcloud_merge_node', 'pointcloud_merger'),
        ('pointcloud_voxel_filter_node', 'pointcloud_voxel_filter'),
        ('pointcloud_colorize_node', 'pointcloud_colorizer'),
        ('image_enhancer.py', 'image_enhancer'),
    ]:
        overrides = {}
        if name == 'lidar_timestamp_shift':
            overrides['auto_timestamp_offset'] = hardware
            if not hardware:
                # Tests/playback supply ROS-clock timestamps. Legacy ROS 1 bags
                # require explicit conversion rather than the old fixed epoch offset.
                overrides['timestamp_offset'] = 0.0
        if name == 'image_enhancer':
            overrides['temperature_path'] = temperature_path
            overrides['temperature_poll_interval'] = float(LaunchConfiguration('temperature_poll_interval').perform(context))
        nodes.append(Node(package='node_pc', executable=executable, name=name,
                          output='screen', parameters=[params, overrides]))
    if hardware:
        camera_mapping = str(share / 'config/blackfly_s.yaml')
        # Parameters preserve the original calibrated 484x366 image geometry.
        # Sensor-side persistent IP is read, never rewritten at startup.
        nodes.extend([
            Node(package='livox_ros2_driver', executable='livox_ros2_driver_node',
                 name='livox_lidar_publisher', output='screen', parameters=[{
                     'xfer_format': 0, 'multi_topic': 0, 'data_src': 0,
                     'publish_freq': 10.0, 'output_data_type': 0,
                     'frame_id': 'livox_frame',
                     'user_config_path': str(share / 'config/livox.json'),
                     'cmdline_input_bd_code': os.getenv('LIVOX_BROADCAST_CODE', '3JEDLB30015Y251'),
                 }]),
            Node(package='spinnaker_camera_driver', executable='camera_driver_node',
                 name='camera', output='screen', parameters=[{
                     'serial_number': os.getenv('FLIR_CAMERA_SERIAL', '24510717'),
                     'parameter_file': camera_mapping, 'frame_id': 'camera',
                     'retry_attempts': 5, 'connect_while_subscribed': False,
                     'buffer_queue_size': 4, 'image_queue_size': 2,
                     'adjust_timestamp': False, 'use_ieee_1588': False,
                     'binning_x': 4, 'binning_y': 4,
                     'pixel_format': 'BayerRG8', 'frame_rate_enable': True, 'frame_rate': 35.0,
                     'gev_scps_packet_size': 1400, 'gev_scpd': 1000,
                     'exposure_auto': 'Continuous', 'auto_exposure_time_upper_limit': 20000.0,
                     'gain_auto': 'Continuous', 'gamma_enable': True, 'gamma': 0.75,
                     'sharpening_enable': True, 'auto_sharpness': True,
                     'saturation_enable': False,
                     'stream_buffer_count_mode': 'Manual', 'stream_buffer_count': 64,
                     'stream_packet_resend_enable': True,
                 }]),
            Node(package='imu_complementary_filter', executable='complementary_filter_node',
                 name='complementary_filter_gain_node', output='screen', parameters=[{
                     'do_bias_estimation': True, 'bias_alpha': 0.05,
                     'do_adaptive_gain': False, 'use_mag': False,
                 }], remappings=[('imu/data_raw', '/livox/imu')]),
        ])
    nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
                      name='static_map_to_livox', output='screen', arguments=[
                          '--x', '0', '--y', '0', '--z', '0', '--roll', '0',
                          '--pitch', '0', '--yaw', '0', '--frame-id', 'map',
                          '--child-frame-id', 'livox_frame']))
    if bridge:
        nodes.append(Node(package='rosbridge_server', executable='rosbridge_websocket',
                          name='rosbridge_websocket', output='screen', parameters=[{
                              'address': LaunchConfiguration('bridge_address').perform(context),
                              'port': int(LaunchConfiguration('bridge_port').perform(context)),
                              'max_message_size': 64000000,
                          }]))
    # Exit the complete launch after an essential process exit. systemd/container
    # supervision owns bounded retry, so stale drivers cannot coexist with retries.
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=[EmitEvent(event=Shutdown(reason='pipeline process exited'))]
    )) for node in nodes]
    return handlers + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='true'),
        DeclareLaunchArgument('rosbridge', default_value='true'),
        DeclareLaunchArgument('bridge_address', default_value=os.getenv('ROSBRIDGE_ADDRESS', '10.20.0.61')),
        DeclareLaunchArgument('bridge_port', default_value=os.getenv('ROSBRIDGE_PORT', '9090')),
        DeclareLaunchArgument('temperature_poll_interval', default_value='30.0'),
        DeclareLaunchArgument('temperature_path', default_value='/sys/class/thermal/thermal_zone0/temp'),
        OpaqueFunction(function=bringup),
    ])
