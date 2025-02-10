import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def load_node_params(yaml_file, node_name):
    """Extract and return parameters for a specific node under /**."""
    with open(yaml_file, 'r') as file:
        full_params = yaml.safe_load(file)
    # Navigate to the parameters under /** and then to the specific node
    node_params = full_params.get('/**', {}).get('ros__parameters', {}).get(node_name, {})

    # return { 'ros__parameters': node_params }
    return node_params


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

    return LaunchDescription([
        robot_namespace_arg,
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            namespace=LaunchConfiguration('namespace'),
            name='rplidar_node',
            output='screen',
            parameters=[rplidar_params],
        ),
    ])
