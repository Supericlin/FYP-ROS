#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('fall_detection_pkg')
    
    return LaunchDescription([
        # HF867 Camera Parameters
        DeclareLaunchArgument(
            'frame_width',
            default_value='320',
            description='Camera frame width for HF867'
        ),
        DeclareLaunchArgument(
            'frame_height',
            default_value='240',
            description='Camera frame height for HF867'
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='30',
            description='Camera FPS target'
        ),
        DeclareLaunchArgument(
            'auto_exposure',
            default_value='false',
            description='Enable auto exposure mode (false = manual mode)'
        ),
        DeclareLaunchArgument(
            'exposure_time',
            default_value='30',
            description='Manual exposure time (lower = less overexposure)'
        ),
        DeclareLaunchArgument(
            'gain',
            default_value='0',
            description='Manual gain setting'
        ),
        DeclareLaunchArgument(
            'brightness',
            default_value='25',
            description='Brightness (0-100, lower for HF867)'
        ),
        DeclareLaunchArgument(
            'contrast',
            default_value='85',
            description='Contrast (0-100, higher for HF867)'
        ),
        DeclareLaunchArgument(
            'saturation',
            default_value='65',
            description='Saturation (0-100, higher for wide-angle)'
        ),
        DeclareLaunchArgument(
            'enable_wide_angle_correction',
            default_value='false',
            description='Enable fisheye correction for 120° FOV'
        ),
        DeclareLaunchArgument(
            'enable_color_correction',
            default_value='false',
            description='Enable color correction for HF867'
        ),
        
        # Fall Detection Parameters
        DeclareLaunchArgument(
            'fall_threshold',
            default_value='0.7',
            description='Aspect ratio threshold for fall detection'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.2',
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
        
        # Preview Parameters
        DeclareLaunchArgument(
            'show_preview',
            default_value='true',
            description='Show image preview window'
        ),
        DeclareLaunchArgument(
            'preview_width',
            default_value='320',
            description='Preview window width'
        ),
        DeclareLaunchArgument(
            'preview_height',
            default_value='240',
            description='Preview window height'
        ),
        
        # Launch HF867 Camera Node
        Node(
            package='fall_detection_pkg',
            executable='camera_node',
            name='hf867_camera',
            parameters=[{
                'frame_width': LaunchConfiguration('frame_width'),
                'frame_height': LaunchConfiguration('frame_height'),
                'fps': LaunchConfiguration('fps'),
                'auto_exposure': LaunchConfiguration('auto_exposure'),
                'exposure_time': LaunchConfiguration('exposure_time'),
                'gain': LaunchConfiguration('gain'),
                'brightness': LaunchConfiguration('brightness'),
                'contrast': LaunchConfiguration('contrast'),
                'saturation': LaunchConfiguration('saturation'),
                #'hue': 0,
                'white_balance': True,
                'enable_color_correction': LaunchConfiguration('enable_color_correction'),
                'enable_wide_angle_correction': LaunchConfiguration('enable_wide_angle_correction'),
                'enable_undistortion': False,
                'camera_matrix_path': os.path.join(pkg_dir, 'calibration_data', 'hf867_camera_matrix.npy'),
                'distortion_coeffs_path': os.path.join(pkg_dir, 'calibration_data', 'hf867_dist_coeffs.npy'),
            }],
            output='screen',
            emulate_tty=True,
        ),
        
        # Launch Fall Detection Node
        Node(
            package='fall_detection_pkg',
            executable='fall_detection_node',
            name='fall_detector',
            parameters=[{
                'model_path': os.path.join(pkg_dir, 'models', 'mobilenet_ssd.tflite'),
                'fall_threshold': LaunchConfiguration('fall_threshold'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'debug_mode': LaunchConfiguration('debug_mode'),
                'mqtt_broker': LaunchConfiguration('mqtt_broker'),
            }],
            output='screen',
            emulate_tty=True,
        ),
        
        # Launch Image Preview Node
        Node(
            package='fall_detection_pkg',
            executable='image_preview_node',
            name='hf867_preview',
            parameters=[{
                'window_name': 'HF867 Fall Detection Preview',
                'window_width': LaunchConfiguration('preview_width'),
                'window_height': LaunchConfiguration('preview_height'),
                'show_fps': True,
                'show_timestamp': True,
                'enable_display': LaunchConfiguration('show_preview'),
            }],
            output='screen',
            emulate_tty=True,
        )
    ]) 