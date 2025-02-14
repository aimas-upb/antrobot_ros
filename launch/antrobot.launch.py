# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.
import os
import re
import platform
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    hostname = platform.node()
    match = re.search(r'-(antrobot\d+)', hostname)
    namespace = match.group(1) if match else ''
    
    namespace_launch_arg = DeclareLaunchArgument('namespace', default_value=namespace, description='Namespace for the robot')
    rdrive_launch_arg = DeclareLaunchArgument('launch_rdrive', default_value='true', description='Launch rdrive')
    rplidar_launch_arg = DeclareLaunchArgument('launch_rplidar', default_value='true', description='Launch rplidar')
    tf_static_link_launch_arg = DeclareLaunchArgument('launch_tf_static_link', default_value='true', description='Launch tf_static_link')
    laserscan_to_pointcloud_launch_arg = DeclareLaunchArgument('launch_laserscan_to_pointcloud', default_value='true', description='Launch laserscan_to_pointcloud')
    kiss_icp_launch_arg = DeclareLaunchArgument('launch_kiss_icp', default_value='true', description='Launch kiss_icp')

    # Include launch files conditionally
    rdrive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('antrobot_ros').find('antrobot_ros'), 'launch', 'rdrive.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_rdrive')),
        launch_arguments={'namespace': LaunchConfiguration('namespace')}.items()
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('antrobot_ros').find('antrobot_ros'), 'launch', 'rplidar.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_rplidar')),
        launch_arguments={'namespace': LaunchConfiguration('namespace')}.items()
    )
    
    tf_static_link = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('antrobot_ros').find('antrobot_ros'), 'launch', 'tf_static_link.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_tf_static_link')),
        launch_arguments={'namespace': LaunchConfiguration('namespace')}.items()
    )

    laserscan_to_pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('antrobot_ros').find('antrobot_ros'), 'launch', 'laserscan_to_pointcloud.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_laserscan_to_pointcloud')),
        launch_arguments={'namespace': LaunchConfiguration('namespace')}.items()
    )

    kiss_icp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('antrobot_ros').find('antrobot_ros'), 'launch', 'kiss_icp.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_kiss_icp')),
        launch_arguments={'namespace': LaunchConfiguration('namespace')}.items()
    )

    return LaunchDescription([
        namespace_launch_arg,
        rdrive_launch_arg,
        rplidar_launch_arg,
        tf_static_link_launch_arg,
        laserscan_to_pointcloud_launch_arg,
        kiss_icp_launch_arg,
        rdrive_launch,
        rplidar_launch,
        tf_static_link,
        laserscan_to_pointcloud_launch,
        kiss_icp_launch,
    ])
