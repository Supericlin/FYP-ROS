from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mqtt_navigation',
            executable='mqtt_navigation_node',
            name='mqtt_navigation_node',
            output='screen',
            parameters=[
                {'mqtt_broker_address': '192.168.0.224'},
                {'mqtt_broker_port': 1883},
                {'mqtt_topic': 'robot/control'}
            ]
        )
    ])