# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from antrobot_ros.utils import load_node_params  
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():   
    # Declare launcher robot namespace argument
    robot_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot instance'
    )
    
    # Load parameters for the tf_static node from antrobot_params.yaml
    config_file_path = os.path.join(
        get_package_share_directory('antrobot_ros'),
        'config',
        'antrobot_params.yaml'
    )
    tf_params = load_node_params(config_file_path, 'tf_static') 

    # Extract transforms list
    transforms = tf_params.get('transforms', [])
    
    publish_transforms = tf_params.get('publish_transforms', '')
    if publish_transforms:
        tf_nodes = []
        for transform in transforms:
            tf_nodes.append(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                namespace=LaunchConfiguration('namespace'),
                arguments=[
                    '--x', str(transform['translation'][0]),
                    '--y', str(transform['translation'][1]),
                    '--z', str(transform['translation'][2]),
                    '--roll', str(transform['rotation'][0]),
                    '--pitch', str(transform['rotation'][1]),
                    '--yaw', str(transform['rotation'][2]),
                    '--frame-id', transform['parent_frame'],
                    '--child-frame-id', transform['child_frame']
                ],
                name=f"tf_static_publisher_{transform['parent_frame']}_{transform['child_frame']}",
                output='screen'
            ))
        return LaunchDescription([robot_namespace_arg] + tf_nodes)
    return LaunchDescription([])
