#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments for logging control
        DeclareLaunchArgument(
            'debug_mode',
            default_value='false',
            description='Enable debug logging for detailed sensor information'
        ),
        DeclareLaunchArgument(
            'log_interval',
            default_value='30.0',
            description='Interval in seconds for status logging (0.0 to disable, optimized: 30s)'
        ),
        DeclareLaunchArgument(
            'enable_detailed_logging',
            default_value='false',
            description='Enable detailed logging of individual sensor readings'
        ),
        
        # Launch the US100 sensor node
        Node(
            package='us100_sensor_pkg',
            executable='us100_sensor_node',
            name='us100_sensor_node',
            parameters=[{
                'debug_mode': LaunchConfiguration('debug_mode'),
                'log_interval': LaunchConfiguration('log_interval'),
                'enable_detailed_logging': LaunchConfiguration('enable_detailed_logging'),
            }],
            output='screen',
            emulate_tty=True,
        )
    ]) 