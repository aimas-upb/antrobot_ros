# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from antrobot_ros.utils import load_node_params

def generate_launch_description():
    # Declare the namespace argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot instance'
    )

    # Get the path to the configuration file
    config_file_path = os.path.join(
        get_package_share_directory('antrobot_ros'),
        'config',
        'antrobot_params.yaml'
    )
    
    # Load the parameters from the configuration file
    params = load_node_params(config_file_path, 'laserscan_to_pointcloud')    


    return LaunchDescription([
        namespace_arg,
        Node(
            package='antrobot_ros',
            executable='laserscan_to_pointcloud_node',
            name='laserscan_to_pointcloud',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[params]
        )
    ])