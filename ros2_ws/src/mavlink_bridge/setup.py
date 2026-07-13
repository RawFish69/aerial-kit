from setuptools import setup
import os
from glob import glob

package_name = 'mavlink_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='MAVLink backend adapter for real PX4/ArduPilot flight controllers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mavlink_bridge_node = mavlink_bridge.mavlink_bridge_node:main',
        ],
    },
)
