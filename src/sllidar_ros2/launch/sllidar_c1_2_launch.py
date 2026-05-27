#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ## front
    channel_type1 = LaunchConfiguration('channel_type1', default='serial')
    serial_port1 = LaunchConfiguration('serial_port1', default='/dev/ttyUSB_LIDAR1_FRONT')
    serial_baudrate1 = LaunchConfiguration('serial_baudrate1', default='460800') # for c1 is 460800
    frame_id1 = LaunchConfiguration('frame_id1', default='lidar1_link')
    inverted1 = LaunchConfiguration('inverted1', default='false')
    angle_compensate1 = LaunchConfiguration('angle_compensate1', default='true')
    scan_mode1 = LaunchConfiguration('scan_mode1', default='DenseBoost')

    ## back
    channel_type2 = LaunchConfiguration('channel_type2', default='serial')
    serial_port2 = LaunchConfiguration('serial_port2', default='/dev/ttyUSB_LIDAR2_REAR')
    serial_baudrate2 = LaunchConfiguration('serial_baudrate2', default='460800') # for c1 is 460800
    frame_id2 = LaunchConfiguration('frame_id2', default='lidar2_link')
    inverted2 = LaunchConfiguration('inverted2', default='false')
    angle_compensate2 = LaunchConfiguration('angle_compensate2', default='true')
    scan_mode2 = LaunchConfiguration('scan_mode2', default='DenseBoost')

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type1',
            default_value=channel_type1,
            description='Specifying channel type of lidar1'),

        DeclareLaunchArgument(
            'serial_port1',
            default_value=serial_port1,
            description='Specifying usb port to connected lidar1'),

        DeclareLaunchArgument(
            'serial_baudrate1',
            default_value=serial_baudrate1,
            description='Specifying usb port baudrate to connected lidar1'),

        DeclareLaunchArgument(
            'frame_id1',
            default_value=frame_id1,
            description='Specifying frame_id of lidar1'),

        DeclareLaunchArgument(
            'inverted1',
            default_value=inverted1,
            description='Specifying whether or not to invert scan data1'),

        DeclareLaunchArgument(
            'angle_compensate1',
            default_value=angle_compensate1,
            description='Specifying whether or not to enable angle_compensate of scan data1'),

        DeclareLaunchArgument(
            'scan_mode1',
            default_value=scan_mode1,
            description='Specifying scan mode of lidar1'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node1',
            namespace='rplidar1',
            parameters=[{'channel_type': channel_type1,
                         'serial_port': serial_port1, 
                         'serial_baudrate': serial_baudrate1, 
                         'frame_id': frame_id1,
                         'inverted': inverted1, 
                         'angle_compensate': angle_compensate1, 
                         'scan_mode': scan_mode1,
                              }],
            # remappings=[('scan', '/scan')],                  
            output='screen'),

        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter_chain',
            namespace='rplidar1',
            parameters=[{
                'filter1': {
                    'name': 'angle1',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': -1.5708,
                        'upper_angle': 1.5708
                    }
                }
            }],
            # remappings=[('rplidar1/scan', '/scan')],
            output='screen'
        ),

        DeclareLaunchArgument(
            'channel_type2',
            default_value=channel_type2,
            description='Specifying channel type of lidar2'),

        DeclareLaunchArgument(
            'serial_port2',
            default_value=serial_port2,
            description='Specifying usb port to connected lidar2'),

        DeclareLaunchArgument(
            'serial_baudrate2',
            default_value=serial_baudrate2,
            description='Specifying usb port baudrate to connected lidar2'),

        DeclareLaunchArgument(
            'frame_id2',
            default_value=frame_id2,
            description='Specifying frame_id of lidar2'),

        DeclareLaunchArgument(
            'inverted2',
            default_value=inverted2,
            description='Specifying whether or not to invert scan data2'),

        DeclareLaunchArgument(
            'angle_compensate2',
            default_value=angle_compensate2,
            description='Specifying whether or not to enable angle_compensate of scan data2'),

        DeclareLaunchArgument(
            'scan_mode2',
            default_value=scan_mode2,
            description='Specifying scan mode of lidar2'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node2',
            namespace='rplidar2',
            parameters=[{'channel_type': channel_type2,
                         'serial_port': serial_port2, 
                         'serial_baudrate': serial_baudrate2, 
                         'frame_id': frame_id2,
                         'inverted': inverted2, 
                         'angle_compensate': angle_compensate2, 
                         'scan_mode': scan_mode2}],
            output='screen'),

        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter_chain',
            namespace='rplidar2',
            parameters=[{
                'filter1': {
                    'name': 'angle1',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': -1.5708,
                        'upper_angle': 1.5708
                    }
                }
            }],
            # remappings=[('rplidar2/scan', '/scan')],
            output='screen'
        ),

    ])

