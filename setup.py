import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'antrobot_ros'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install the config folder and its contents
        ('share/' + package_name + '/config', ['config/antrobot_params.yaml']),
        ('share/' + package_name + '/config', ['config/keyboard_teleoperation_params.yaml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        
    ],
    install_requires=['setuptools', 'smbus2', 'pyyaml'],
    zip_safe=True,
    maintainer='Dan Novischi',
    maintainer_email='dan_marius.novischi@upb.ro',
    description='This package provides ROS2 nodes and interfaces to control the Antrobot.',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rdrive_node = antrobot_ros.rdrive_node:main',
            'keyboard_teleoperation_node = antrobot_ros.keyboard_teleoperation_node:main'
        ],
    },
    
    package_data={
        package_name: ['config/antrobot_params.yaml','config/keyboard_teleoperation_params.yaml'],
    },
    scripts=[
        'antrobot_ros/rdrive_node.py',
        'antrobot_ros/keyboard_teleoperation_node.py'
    ],
)
