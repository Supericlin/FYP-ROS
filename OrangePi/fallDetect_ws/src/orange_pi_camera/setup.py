from setuptools import setup
import os

package_name = 'orange_pi_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'models'), 
            ['models/mobilenet_ssd.tflite']),
        (os.path.join('share', package_name, 'launch'), 
            ['launch/camera_launch.py',
             'launch/fall_detection_launch.py',
             'launch/complete_system_launch.py']),
    ],
    install_requires=[
        'setuptools',
        'numpy',              # For array handling
        'paho-mqtt',          # MQTT client
        'tflite-runtime',     # TensorFlow Lite
        'mediapipe',          # New: Hand tracking
        'opencv-python-headless'  # New: OpenCV without GUI
    ],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@domain.com',
    description='Camera package for Orange Pi',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'camera_node = orange_pi_camera.camera_node:main',
            'fall_detection_node = orange_pi_camera.fall_detection_node:main',
        ],
    },
)