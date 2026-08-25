from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command

import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to your config file
    minibot_localization_pkg = get_package_share_directory('minibot_slam')
    zed_config_path = os.path.join(minibot_localization_pkg, 'config', 'zed2i.yaml')

    minibot_description_pkg = get_package_share_directory('minibot_description')
    minibot_urdf_path = os.path.join(minibot_description_pkg, 'urdf', 'minibot_minimal.urdf.xacro')
    robot_desc = xacro.process_file(minibot_urdf_path).toxml()

    base_link_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc
        }]
    )

    zed_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('zed_wrapper'), '/launch/zed_camera.launch.py']
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'ros_params_override_path': zed_config_path,  # Pass your optimized config
            'publish_tf': 'false',
            'publish_map_tf': 'false',
        }.items()
    )

    return LaunchDescription([
        base_link_state_publisher,
        zed_launch_file,
    ])

