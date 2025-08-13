from setuptools import find_packages, setup

package_name = 'us100_sensor_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eric Lin',
    maintainer_email='supoeric@yahoo.com',
    description='Ultrasonic sensors node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'us100_sensor_node = us100_sensor_pkg.us100_sensor_node:main',
        ],
    },
)
