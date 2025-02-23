#!/usr/bin/env python3
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

    joint_state_estimator_params = load_node_params(config_file_path, 'joint_state_estimator')
    
    joint_state_estimator_node = Node(
        package='antrobot_ros',
        executable='joint_state_estimator_node',
        namespace=LaunchConfiguration('namespace'),
        name='joint_state_estimator',
        output='screen',
        parameters=[joint_state_estimator_params],
    )

    return LaunchDescription([robot_namespace_arg, joint_state_estimator_node])
