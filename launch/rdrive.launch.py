# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.
import os
# import re
# import platform
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from antrobot_ros.utils import load_node_params


def generate_launch_description():
    # TODO: Move the hostname extraction to the aggregate luaunch file
    # # Extract the plaform hostname
    # hostname = platform.node()
    # # Obtain the robot base name (e.g. antrobot1, antrobot2, antrobot3...)
    # match = re.search(r'-(antrobot\d+)', hostname)
    # # If no match default to antrobot1
    # namespace = match.group(1) if match else 'antrobot1'

    robot_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot instance'
    )
 
    config_file_path = os.path.join(
        get_package_share_directory('antrobot_ros'),
        'config',
        'antrobot_params.yaml'
    )
    
    rdrive_parmas = load_node_params(config_file_path, 'rdrive')
    
   # Create the rdrive node
    rdrive_node = Node(
        package='antrobot_ros',
        executable='rdrive_node.py',
        namespace=LaunchConfiguration('namespace'),
        name='rdrive_node',
        parameters=[rdrive_parmas]
    )
    
    return LaunchDescription([robot_namespace_arg, rdrive_node ])
