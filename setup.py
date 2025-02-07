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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dan Novischi',
    maintainer_email='dan_marius.novischi@upb.ro',
    description='This package provides ROS2 nodes and interfaces to control the Antrobot.',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
