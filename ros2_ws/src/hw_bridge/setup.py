from glob import glob
import os

from setuptools import setup

package_name = 'hw_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='Hardware flight backend (CRSF/Betaflight + PX4/MAVLink)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crsf_backend_adapter_node = hw_bridge.crsf_backend_adapter_node:main',
            'hw_state_estimator_node = hw_bridge.hw_state_estimator_node:main',
            'px4_backend_adapter_node = hw_bridge.px4_backend_adapter_node:main',
        ],
    },
)
