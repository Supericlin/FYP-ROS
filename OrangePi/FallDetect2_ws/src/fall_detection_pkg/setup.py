from setuptools import setup
import os
from glob import glob

package_name = 'fall_detection_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Models directory
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        # Calibration data directory
        (os.path.join('share', package_name, 'calibration_data'), glob('calibration_data/*')),
        # Scripts directory
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    python_requires='>=3.8',
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Fall detection package for HF867 wide-angle camera',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = fall_detection_pkg.camera_node:main',
            'fall_detection_node = fall_detection_pkg.fall_detection_node:main',
            'image_preview_node = fall_detection_pkg.image_preview_node:main',
            'test_node = fall_detection_pkg.test_node:main',
        ],
    },
)
