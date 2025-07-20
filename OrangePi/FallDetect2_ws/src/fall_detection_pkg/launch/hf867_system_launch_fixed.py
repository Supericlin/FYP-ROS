#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
from pathlib import Path
import os

def generate_launch_description():
    # Get the directory where this launch file is located
    pkg_dir = Path(__file__).parent
    
    # Get the workspace install directory
    workspace_dir = Path(__file__).parent.parent.parent.parent
    install_dir = workspace_dir / "install" / "fall_detection_pkg"
    
    # Define executable paths
    camera_executable = str(install_dir / "bin" / "camera_node")
    fall_detection_executable = str(install_dir / "bin" / "fall_detection_node")
    
    return LaunchDescription([
        # HF867 Camera specific parameters
        DeclareLaunchArgument(
            'camera_device',
            default_value='0',
            description='Camera device number for HF867'
        ),
        DeclareLaunchArgument(
            'frame_width',
            default_value='640',
            description='Frame width for HF867 (640 recommended)'
        ),
        DeclareLaunchArgument(
            'frame_height',
            default_value='480',
            description='Frame height for HF867 (480 recommended)'
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='30',
            description='Frame rate for HF867 (30fps recommended)'
        ),
        DeclareLaunchArgument(
            'enable_undistortion',
            default_value='true',
            description='Enable undistortion for 120° wide-angle lens'
        ),
        DeclareLaunchArgument(
            'fall_threshold',
            default_value='0.7',
            description='Fall detection threshold for 120° wide-angle (0.7 recommended)'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.25',
            description='Person detection confidence for wide-angle (0.25 recommended)'
        ),
        DeclareLaunchArgument(
            'debug_mode',
            default_value='true',
            description='Enable debug logging for tuning'
        ),
        DeclareLaunchArgument(
            'mqtt_broker',
            default_value='192.168.0.224',
            description='MQTT broker IP address'
        ),
        
        # Launch both nodes with HF867 optimized settings
        GroupAction([
            # Camera node optimized for HF867
            Node(
                package='fall_detection_pkg',
                executable=camera_executable,
                name='hf867_camera',
                parameters=[{
                    'camera_device': LaunchConfiguration('camera_device'),
                    'frame_width': LaunchConfiguration('frame_width'),
                    'frame_height': LaunchConfiguration('frame_height'),
                    'fps': LaunchConfiguration('fps'),
                    'enable_undistortion': LaunchConfiguration('enable_undistortion'),
                    'camera_matrix_path': str(pkg_dir / 'calibration_data' / 'hf867_camera_matrix.npy'),
                    'distortion_coeffs_path': str(pkg_dir / 'calibration_data' / 'hf867_dist_coeffs.npy'),
                    'buffer_size': 3,
                    'auto_exposure': True,
                }],
                output='screen',
                emulate_tty=True,
            ),
            
            # Fall detection node optimized for HF867
            Node(
                package='fall_detection_pkg',
                executable=fall_detection_executable,
                name='hf867_fall_detector',
                parameters=[{
                    'fall_threshold': LaunchConfiguration('fall_threshold'),
                    'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                    'debug_mode': LaunchConfiguration('debug_mode'),
                    'mqtt_broker': LaunchConfiguration('mqtt_broker'),
                    'model_path': 'models/mobilenet_ssd.tflite',
                }],
                output='screen',
                emulate_tty=True,
            ),
        ])
    ]) 