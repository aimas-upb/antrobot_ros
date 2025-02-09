from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the namespace argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the keyboard teleoperation node'
    )
    
    print(namespace_arg)

    # Get the path to the configuration file
    config_file_path = os.path.join(
        get_package_share_directory('antrobot_ros'),
        'config',
        'keyboard_teleoperation_params.yaml'
    )
    
    return LaunchDescription([
        namespace_arg,
        Node(
            package='antrobot_ros',
            executable='keyboard_teleoperation_node',
            name='keyboard_teleoperation',
            namespace=LaunchConfiguration('namespace'),
            parameters=[config_file_path],
            output='screen',
            emulate_tty=True,
            prefix='xterm -e'    
        )
    ])