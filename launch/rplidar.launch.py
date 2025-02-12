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
    # Declare launcher robot namespace argument
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

    rplidar_params = load_node_params(config_file_path, 'rplidar')
    
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        namespace=LaunchConfiguration('namespace'),
        name='rplidar_node',
        output='screen',
        parameters=[rplidar_params],
    )

    return LaunchDescription([ robot_namespace_arg, rplidar_node ])
