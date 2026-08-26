from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

# ---------------------------------------------------------------------
# Hardcode the bag you're tuning against here. Change ONLY this line
# when you record a new bag -- everything else in this file stays fixed
# so every replay changes parameters only, never the underlying data.
# ---------------------------------------------------------------------
BAG_PATH = '/home/YOUR_USER/bags/mapping_run_01'


def generate_launch_description():
    """
    Bag replay + ekf + rtabmap, for TUNING.

    Plays back the raw topics + /tf_static recorded by
    record_bringup.launch.py, and runs ekf_filter_node + rtabmap fresh
    against that replayed data with use_sim_time:=true, so they run in
    lockstep with the bag's recorded timestamps rather than wall clock.

    rtabmap reads config/rtabmap_params.yaml -- the SAME file
    localization.launch.py uses for the real robot -- so edit that file,
    relaunch this file, inspect the result, repeat. Once you're happy,
    the real robot picks up the same values automatically.
    """
    package_name = 'minibot_slam'
    package_share_dir = get_package_share_directory(package_name)

    ekf_config = os.path.join(package_share_dir, 'config', 'ekf_config.yaml')
    rtabmap_config = os.path.join(package_share_dir, 'config', 'rtabmap_params.yaml')

    # --clock publishes /clock from the bag's recorded timestamps so every
    # use_sim_time:=true node below advances with the bag, not the wall clock.
    # Delayed slightly so ekf/rtabmap are already up and subscribed before
    # the first messages arrive.
    bag_play = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'play', BAG_PATH, '--clock'],
            output='screen'
        )]
    )

    # Sole publisher of odom -> base_link during replay.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        remappings=[('odometry/filtered', '/odom_fused')]
    )

    remappings = [
        ('rgb/image',       '/zed/zed_node/rgb/color/rect/image'),
        ('rgb/camera_info', '/zed/zed_node/rgb/color/rect/camera_info'),
        ('depth/image',     '/zed/zed_node/depth/depth_registered'),
        ('odom',            '/odom_fused'),
        ('imu',             '/zed/zed_node/imu/data'),
    ]

    # Sole publisher of map -> odom during replay.
    # '-d' deletes any previous rtabmap.db so each replay starts clean --
    # otherwise rtabmap loads the old map in localization mode instead of
    # mapping fresh, and you'd be comparing against stale data.
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[rtabmap_config, {'use_sim_time': True}],
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
            'use_sim_time': True,
        }],
        remappings=remappings
    )

    return LaunchDescription([
        ekf_node,
        rtabmap_node,
        rtabmap_viz_node,
        bag_play,
    ])
