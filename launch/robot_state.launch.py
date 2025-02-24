import os
import xacro
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

    robot_state_params = load_node_params(config_file_path, 'robot_state')
    
    # Extract the URDF from the xacro file
    share_dir = get_package_share_directory(robot_state_params['description_package'])
    xacro_file = os.path.join(share_dir, 'urdf', robot_state_params['description_file'])
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # The frame prefix might be set in the param config file for the robot
    # If not use the namespace as the frame prefix
    if not robot_state_params['frame_prefix']:
        robot_state_params['frame_prefix'] = LaunchConfiguration('namespace')

    # Create the parameters for the robot_state_publisher node
    robot_state_publisher_params = {
        'robot_description': robot_urdf,
        'publish_frequency': robot_state_params['publish_frequency'],
        'ignore_timestamp': robot_state_params['ignore_timestamp'],
        'frame_prefix': robot_state_params['frame_prefix']
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_state_publisher_params],
        # remappings=[('/joint_states', robot_state_params['joint_states_topic'])]
    )

    return LaunchDescription([
        robot_namespace_arg,
        robot_state_publisher_node
    ])