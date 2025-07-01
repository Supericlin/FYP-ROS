from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mqtt_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'paho-mqtt', 'action_msgs', 'nav2_msgs', 'lifecycle_msgs'],
    zip_safe=True,
    maintainer='Eric Lin',
    maintainer_email='supoeric@yahoo.com',
    description='MQTT to ROS2 Nav2 bridge node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_navigation_node = mqtt_navigation.mqtt_navigation_node:main',
        ],
    },
)