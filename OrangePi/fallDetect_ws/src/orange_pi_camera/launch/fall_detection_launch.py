#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # MQTT Parameters
        DeclareLaunchArgument(
            'mqtt_broker',
            default_value='192.168.0.224',
            description='MQTT broker IP address'
        ),
        
        # Model Parameters
        DeclareLaunchArgument(
            'model_path',
            default_value='models/mobilenet_ssd.tflite',
            description='Path to the TFLite model file'
        ),
        
        # Fall Detection Node
        Node(
            package='orange_pi_camera',
            executable='fall_detection_node',
            name='fall_detector',
            parameters=[{
                'mqtt_broker': LaunchConfiguration('mqtt_broker'),
                'model_path': LaunchConfiguration('model_path'),
            }],
            output='screen',
            emulate_tty=True,
        )
    ]) 