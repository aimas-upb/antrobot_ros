# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from antrobot_ros.utils import load_node_params

def generate_launch_description():
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

    kiss_icp_params = load_node_params(config_file_path, 'kiss_icp')
    
    laserscan_to_pointcloud_params = load_node_params(config_file_path, 'laserscan_to_pointcloud')
    
    pointcloud_topic = laserscan_to_pointcloud_params['pointcloud_topic']
    
    kiss_icp_node = Node(
        package='kiss_icp',
        executable='kiss_icp_node',
        namespace=LaunchConfiguration('namespace'),
        name='kiss_icp',
        output='screen',
        parameters=[kiss_icp_params],
        remappings=[
            ("pointcloud_topic", pointcloud_topic),
        ],
    )

    return LaunchDescription([robot_namespace_arg, kiss_icp_node])
