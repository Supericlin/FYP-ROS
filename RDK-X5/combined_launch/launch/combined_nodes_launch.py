#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

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
            default_value='10.0',
            description='Interval in seconds for status logging (0.0 to disable)'
        ),
        DeclareLaunchArgument(
            'enable_detailed_logging',
            default_value='false',
            description='Enable detailed logging of individual sensor readings'
        ),
        DeclareLaunchArgument(
            'enable_home_labels',
            default_value='true',
            description='Enable home label publishing'
        ),
        DeclareLaunchArgument(
            'enable_us100_sensor',
            default_value='true',
            description='Enable US100 sensor node'
        ),
        DeclareLaunchArgument(
            'enable_mqtt_navigation',
            default_value='true',
            description='Enable MQTT navigation node'
        ),
        
        # MQTT Navigation parameters
        DeclareLaunchArgument(
            'mqtt_broker_address',
            default_value='192.168.0.224',
            description='MQTT broker IP address'
        ),
        DeclareLaunchArgument(
            'mqtt_broker_port',
            default_value='1883',
            description='MQTT broker port'
        ),
        DeclareLaunchArgument(
            'mqtt_topic',
            default_value='robot/control',
            description='MQTT topic for receiving commands'
        ),
        DeclareLaunchArgument(
            'mqtt_status_topic',
            default_value='robot/status',
            description='MQTT topic for publishing status'
        ),
        
        # MQTT Navigation Control Function Parameters
        DeclareLaunchArgument(
            'enable_nav_stop',
            default_value='true',
            description='Enable navStop command in MQTT navigation'
        ),
        DeclareLaunchArgument(
            'enable_nav_pause',
            default_value='true',
            description='Enable navPause command in MQTT navigation'
        ),
        DeclareLaunchArgument(
            'enable_nav_resume',
            default_value='true',
            description='Enable navCon (resume) command in MQTT navigation'
        ),
        
        # Launch the home label publisher node
        Node(
            package='home_label_package',
            executable='home_label_node',
            name='home_label_node',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('enable_home_labels')),
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
            condition=IfCondition(LaunchConfiguration('enable_us100_sensor')),
        ),
        
        # Launch the MQTT navigation node
        Node(
            package='mqtt_navigation',
            executable='mqtt_navigation_node',
            name='mqtt_navigation_node',
            parameters=[{
                'mqtt_broker_address': LaunchConfiguration('mqtt_broker_address'),
                'mqtt_broker_port': LaunchConfiguration('mqtt_broker_port'),
                'mqtt_topic': LaunchConfiguration('mqtt_topic'),
                'mqtt_status_topic': LaunchConfiguration('mqtt_status_topic'),
                'enable_nav_stop': LaunchConfiguration('enable_nav_stop'),
                'enable_nav_pause': LaunchConfiguration('enable_nav_pause'),
                'enable_nav_resume': LaunchConfiguration('enable_nav_resume'),
            }],
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('enable_mqtt_navigation')),
        )
    ]) 