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
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot instance'
    )
    
    config_file_path = os.path.join(
        get_package_share_directory('antrobot_ros'),
        'config',
        'antrobot_params.yaml'
    )
    
    cartographer_params = load_node_params(config_file_path, 'cartographer')
    cartographer_config_file_path = os.path.join(
        get_package_share_directory('antrobot_ros'),
        'config',
        cartographer_params['config_file']
    )
    
    

    # This subscribes to the /scan  and /odom topics by default
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        namespace=LaunchConfiguration('namespace'),
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }],
        arguments=[
            '-configuration_directory', os.path.dirname(cartographer_config_file_path),
            '-configuration_basename', os.path.basename(cartographer_config_file_path)
        ]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        namespace=LaunchConfiguration('namespace'),
        name='occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }],
        arguments=[
            '-resolution', str(cartographer_params['resolution']),
            '-publish_period_sec', str(cartographer_params['publish_period_sec'])
        ]
    )

    return LaunchDescription([
        namespace_arg,
        cartographer_node,
        occupancy_grid_node
    ])