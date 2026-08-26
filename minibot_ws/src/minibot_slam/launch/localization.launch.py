from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = 'minibot_slam'
    package_share_dir = get_package_share_directory(package_name)

    # ---------------------------------------------------------
    # 1. Wheel odometry
    #    Publishes /wheel/odom (topic only). The node in
    #    minibot_odometry does NOT also broadcast odom -> base_link --
    #    the EKF below is the sole owner of that transform.
    # ---------------------------------------------------------
    wheel_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('minibot_odometry'), '/launch/odometry.launch.py']
        )
    )

    # ---------------------------------------------------------
    # 2. Extended Kalman Filter (Sensor Fusion)
    #    Sole publisher of odom -> base_link, fusing wheel + visual odometry.
    # ---------------------------------------------------------
    ekf_config = os.path.join(package_share_dir, 'config', 'ekf_config.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', '/odom_fused')]
    )

    # ---------------------------------------------------------
    # 3. SLAM (RTAB-Map)
    #    Sole publisher of map -> odom.
    #    Reads the SAME rtabmap_params.yaml that tuning_replay.launch.py
    #    uses -- whatever you tune against the bag is exactly what runs
    #    here, with no separate copy to keep in sync.
    # ---------------------------------------------------------
    rtabmap_config = os.path.join(package_share_dir, 'config', 'rtabmap_params.yaml')

    # ---------------------------------------------------------
    # 4. Topic Remappings (Shared by SLAM and VIZ)
    # ---------------------------------------------------------
    remappings = [
        ('rgb/image',       '/zed/zed_node/rgb/color/rect/image'),
        ('rgb/camera_info', '/zed/zed_node/rgb/color/rect/camera_info'),
        ('depth/image',     '/zed/zed_node/depth/depth_registered'),
        ('odom',            '/odom_fused'),  # RTAB-Map listens to EKF output
        ('imu',             '/zed/zed_node/imu/data'),
    ]

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[rtabmap_config],
        remappings=remappings,
        arguments=['-d']
    )

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_odom_info': False,
            'approx_sync': True,
            'wait_for_transform': 0.5,
        }],
        remappings=remappings
    )

    return LaunchDescription([
        wheel_odom_launch,
        ekf_node,
        rtabmap_node,
        rtabmap_viz_node,
    ])
