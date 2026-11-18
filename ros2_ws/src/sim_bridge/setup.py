from setuptools import setup
import os
from glob import glob

package_name = 'sim_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
('share/ament_index/resource_index/packages', ['resource/' + package_name]),
('share/' + package_name, ['package.xml']),
(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
(os.path.join('share', package_name, 'config'), glob('config/*')),
(os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
(os.path.join('share', package_name, 'models', 'x3_velocity_control'), glob('models/x3_velocity_control/*')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='Simulation backend adapters for Gazebo and fast sim',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_backend_adapter_node = sim_bridge.gazebo_backend_adapter_node:main',
            'fastsim_backend_adapter_node = sim_bridge.fastsim_backend_adapter_node:main',
            'fixedwing_backend_adapter_node = sim_bridge.fixedwing_backend_adapter_node:main',
            'multirotor_backend_adapter_node = sim_bridge.multirotor_backend_adapter_node:main',
            'gazebo_fixedwing_pose_node = sim_bridge.gazebo_fixedwing_pose_node:main',
            'gazebo_multirotor_pose_node = sim_bridge.gazebo_multirotor_pose_node:main',
            'gz_smoke_cmd_node = sim_bridge.gz_smoke_cmd_node:main',
        ],
    },
)
