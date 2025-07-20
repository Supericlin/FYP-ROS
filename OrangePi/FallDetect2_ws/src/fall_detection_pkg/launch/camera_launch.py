#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments for camera configuration
        DeclareLaunchArgument(
            'camera_device',
            default_value='0',
            description='Camera device number (usually 0 for first camera)'
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
            description='Camera frame rate (FPS)'
        ),
        DeclareLaunchArgument(
            'enable_undistortion',
            default_value='false',
            description='Enable camera undistortion for wide-angle lenses'
        ),
        DeclareLaunchArgument(
            'camera_matrix_path',
            default_value='',
            description='Path to camera matrix file (.npy)'
        ),
        DeclareLaunchArgument(
            'distortion_coeffs_path',
            default_value='',
            description='Path to distortion coefficients file (.npy)'
        ),
        DeclareLaunchArgument(
            'auto_exposure',
            default_value='true',
            description='Enable auto exposure'
        ),
        #DeclareLaunchArgument(
        #    'exposure_time',
        #    default_value='-1',
        #    description='Manual exposure time (-1 for auto)'
        #),
        
        # Launch the camera node with parameters
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
                'camera_matrix_path': LaunchConfiguration('camera_matrix_path'),
                'distortion_coeffs_path': LaunchConfiguration('distortion_coeffs_path'),
                'auto_exposure': LaunchConfiguration('auto_exposure'),
                #'exposure_time': LaunchConfiguration('exposure_time'),
            }],
            output='screen',
            emulate_tty=True,
        )
    ]) 