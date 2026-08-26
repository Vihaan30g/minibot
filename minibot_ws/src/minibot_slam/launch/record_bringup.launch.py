from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Bring-up for RECORDING a rosbag ONLY.

    Starts everything that PUBLISHES raw topics -- camera, depth, VIO,
    wheel odometry, and the static base_link->zed_camera_link transform
    -- and nothing that CONSUMES them. No ekf_filter_node, no rtabmap.

    This is intentional: the bag must hold raw inputs only, so it can be
    replayed later against as many different ekf/rtabmap parameter sets
    as you like, always against the exact same physical run.

    Usage:
      Terminal 1:  ros2 launch minibot_slam record_bringup.launch.py
      Terminal 2:  ros2 bag record -o ~/bags/mapping_run_01 \\
                     /zed/zed_node/rgb/color/rect/image \\
                     /zed/zed_node/rgb/color/rect/camera_info \\
                     /zed/zed_node/depth/depth_registered \\
                     /zed/zed_node/imu/data \\
                     /zed/zed_node/odom \\
                     /wheel/odom \\
                     /tf_static

    Recording is kept as a separate command (not baked into this launch
    file) on purpose -- you'll want a fresh, uniquely-named bag for each
    physical run, and `-o` makes that a one-word change per session
    instead of an edit to this file.

    Do NOT record /tf: odom->base_link (ekf) and map->odom (rtabmap) get
    regenerated fresh from raw data every time you replay, by design.
    """

    minibot_localization_pkg = get_package_share_directory('minibot_slam')
    zed_config_path = os.path.join(minibot_localization_pkg, 'config', 'zed2i.yaml')

    minibot_description_pkg = get_package_share_directory('minibot_description')
    minibot_urdf_path = os.path.join(minibot_description_pkg, 'urdf', 'minibot_minimal.urdf.xacro')
    robot_desc = xacro.process_file(minibot_urdf_path).toxml()

    # Publishes ONLY base_link -> zed_camera_link. zed_camera_link downward
    # comes from the ZED wrapper's own robot_state_publisher (publish_urdf
    # below) -- don't add a second static tf publisher for those frames.
    base_link_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('zed_wrapper'), '/launch/zed_camera.launch.py']
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'ros_params_override_path': zed_config_path,
            'publish_tf': 'false',        # EKF will own odom -> base_link on replay
            'publish_map_tf': 'false',    # rtabmap will own map -> odom on replay
            'publish_urdf': 'true',
        }.items()
    )

    # Publishes /wheel/odom topic only. Make sure the node in
    # minibot_odometry does NOT also broadcast odom -> base_link --
    # even during recording, keep the bag TF-clean.
    wheel_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('minibot_odometry'), '/launch/odometry.launch.py']
        )
    )

    return LaunchDescription([
        base_link_state_publisher,
        zed_launch,
        wheel_odom_launch,
    ])
