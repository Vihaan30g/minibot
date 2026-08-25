from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

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

    # Publishes ONLY base_link -> zed_camera_link (see the xacro).
    # zed_camera_link downward is published by the ZED wrapper's own
    # robot_state_publisher below -- do not add any more static
    # transform publishers for camera-internal frames anywhere.
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
            'ros_params_override_path': zed_config_path,
            'publish_tf': 'false',        # EKF owns odom -> base_link
            'publish_map_tf': 'false',    # rtabmap owns map -> odom
            'publish_urdf': 'true',       # ZED's own robot_state_publisher owns
                                           # zed_camera_link -> optical/imu frames
        }.items()
    )

    return LaunchDescription([
        base_link_state_publisher,
        zed_launch_file,
    ])