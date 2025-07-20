#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Camera parameters
        DeclareLaunchArgument(
            'camera_device',
            default_value='0',
            description='Camera device number'
        ),
        DeclareLaunchArgument(
            'frame_width',
            default_value='640',
            description='Camera frame width'
        ),
        DeclareLaunchArgument(
            'frame_height',
            default_value='480',
            description='Camera frame height'
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='30',
            description='Camera frame rate'
        ),
        DeclareLaunchArgument(
            'enable_undistortion',
            default_value='false',
            description='Enable camera undistortion'
        ),
        
        # Fall detection parameters (optimized for wide-angle)
        DeclareLaunchArgument(
            'fall_threshold',
            default_value='0.6',
            description='Aspect ratio threshold for fall detection'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.3',
            description='Minimum confidence for person detection'
        ),
        DeclareLaunchArgument(
            'debug_mode',
            default_value='true',
            description='Enable debug logging'
        ),
        DeclareLaunchArgument(
            'mqtt_broker',
            default_value='192.168.0.224',
            description='MQTT broker IP address'
        ),
        
        # Launch both nodes
        GroupAction([
            # Camera node
            Node(
                package='fall_detection_pkg',
                executable='camera_node',
                name='camera_publisher',
                parameters=[{
                    'camera_device': LaunchConfiguration('camera_device'),
                    'frame_width': LaunchConfiguration('frame_width'),
                    'frame_height': LaunchConfiguration('frame_height'),
                    'fps': LaunchConfiguration('fps'),
                    'enable_undistortion': LaunchConfiguration('enable_undistortion'),
                }],
                output='screen',
                emulate_tty=True,
            ),
            
            # Fall detection node
            Node(
                package='fall_detection_pkg',
                executable='fall_detection_node',
                name='fall_detector',
                parameters=[{
                    'fall_threshold': LaunchConfiguration('fall_threshold'),
                    'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                    'debug_mode': LaunchConfiguration('debug_mode'),
                    'mqtt_broker': LaunchConfiguration('mqtt_broker'),
                }],
                output='screen',
                emulate_tty=True,
            ),
        ])
    ]) 