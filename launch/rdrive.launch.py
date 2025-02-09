# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.
import platform
import re
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Extract the plaform hostname
    hostname = platform.node()
    # Obtain the robot base name (e.g. antrobot1, antrobot2, antrobot3...)
    match = re.search(r'-(antrobot\d+)', hostname)
    # If no match default to antrobot1
    namespace = match.group(1) if match else 'antrobot1'

    # Declare launcher robot namespace argument
    robot_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=namespace,
        description='Namespace for the robot instance'
    )
 
    # Get the path to the configuration file
    config_file_path = os.path.join(
        get_package_share_directory('antrobot_ros'),
        'config',
        'antrobot_params.yaml'
    )
    
   # Create the rdrive node
    rdrive_node = Node(
        package='antrobot_ros',
        executable='rdrive_node.py',
        namespace=robot_namespace_arg.default_value,
        name='rdrive_node',
        parameters=[config_file_path]
    )
    
    return LaunchDescription([ rdrive_node ])
