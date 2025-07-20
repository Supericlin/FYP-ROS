#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments for easy tuning
        DeclareLaunchArgument(
            'fall_threshold',
            default_value='0.7',
            description='Aspect ratio threshold for fall detection (0.1-2.0)'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.4',
            description='Minimum confidence for person detection (0.1-1.0)'
        ),
        DeclareLaunchArgument(
            'debug_mode',
            default_value='false',
            description='Enable debug logging (set to true for detailed logs)'
        ),
        DeclareLaunchArgument(
            'mqtt_broker',
            default_value='192.168.0.224',
            description='MQTT broker IP address'
        ),
        DeclareLaunchArgument(
            'frame_width',
            default_value='320',
            description='Camera frame width (lower = better FPS)'
        ),
        DeclareLaunchArgument(
            'frame_height',
            default_value='240',
            description='Camera frame height (lower = better FPS)'
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='30',
            description='Camera FPS target'
        ),
        
        # Launch the camera node with optimized settings
        Node(
            package='fall_detection_pkg',
            executable='camera_node',
            name='camera_publisher',
            parameters=[{
                'frame_width': LaunchConfiguration('frame_width'),
                'frame_height': LaunchConfiguration('frame_height'),
                'fps': LaunchConfiguration('fps'),
                'enable_undistortion': False,  # Disabled for performance
                #'enable_color_correction': False,  # Disabled for performance
                #'auto_exposure': True,
                #'brightness': 50,
                #'contrast': 50,
                #'saturation': 50,
            }],
            output='screen',
            emulate_tty=True,
        ),
        
        # Launch the fall detection node with parameters
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
        )
    ]) 