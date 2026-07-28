import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('minibot_control')
    default_config_path = os.path.join(pkg_share, 'config', 'minibot_controller.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_config_path,
        description='Absolute path to the YAML parameter configuration file.'
    )

    minibot_controller_node = Node(
        package='minibot_control',
        executable='minibot_controller_node',
        name='minibot_controller',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/wheel_cmd_vel', '/wheel_cmd_vel')
        ]
    )

    return LaunchDescription([
        params_file_arg,
        minibot_controller_node
    ])