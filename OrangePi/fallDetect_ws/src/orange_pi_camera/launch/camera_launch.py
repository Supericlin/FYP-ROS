#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Camera Parameters
        DeclareLaunchArgument(
            'camera_index',
            default_value='0',
            description='Camera device index'
        ),
        DeclareLaunchArgument(
            'frame_width',
            default_value='320',
            description='Camera frame width'
        ),
        DeclareLaunchArgument(
            'frame_height',
            default_value='240',
            description='Camera frame height'
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='15',
            description='Camera FPS target'
        ),
        DeclareLaunchArgument(
            'buffer_size',
            default_value='2',
            description='Camera buffer size'
        ),
        DeclareLaunchArgument(
            'enable_auto_reconnect',
            default_value='true',
            description='Enable automatic camera reconnection'
        ),
        DeclareLaunchArgument(
            'reconnect_delay',
            default_value='2.0',
            description='Delay between reconnection attempts (seconds)'
        ),
        DeclareLaunchArgument(
            'max_reconnect_attempts',
            default_value='5',
            description='Maximum number of reconnection attempts'
        ),
        DeclareLaunchArgument(
            'debug_mode',
            default_value='false',
            description='Enable debug logging'
        ),
        
        # Camera Node
        Node(
            package='orange_pi_camera',
            executable='camera_node',
            name='camera_publisher',
            parameters=[{
                'camera_index': LaunchConfiguration('camera_index'),
                'frame_width': LaunchConfiguration('frame_width'),
                'frame_height': LaunchConfiguration('frame_height'),
                'fps': LaunchConfiguration('fps'),
                'buffer_size': LaunchConfiguration('buffer_size'),
                'enable_auto_reconnect': LaunchConfiguration('enable_auto_reconnect'),
                'reconnect_delay': LaunchConfiguration('reconnect_delay'),
                'max_reconnect_attempts': LaunchConfiguration('max_reconnect_attempts'),
                'debug_mode': LaunchConfiguration('debug_mode'),
            }],
            output='screen',
            emulate_tty=True,
        )
    ]) 