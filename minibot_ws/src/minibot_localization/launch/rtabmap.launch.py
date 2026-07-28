from launch import LaunchDescription
from launch.actions import (
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetRemap

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    # ============================================================
    # Official RTAB-Map launch file
    # ============================================================

    rtabmap_launch = os.path.join(
        get_package_share_directory('rtabmap_launch'),
        'launch',
        'rtabmap.launch.py'
    )

    # ============================================================
    # Topic remappings
    #
    # Left  = topic expected internally by RTAB-Map
    # Right = actual topic on Minibot
    # ============================================================

    remappings = [
        (
            'rgb/image',
            '/zed/zed_node/rgb/color/rect/image'
        ),
        (
            'rgb/camera_info',
            '/zed/zed_node/rgb/color/rect/camera_info'
        ),
        (
            'depth/image',
            '/zed/zed_node/depth/depth_registered'
        ),
        (
            'odom',
            '/zed/zed_node/odom'
        ),
        (
            'imu',
            '/zed/zed_node/imu/data'
        ),
    ]

    # ============================================================
    # RTAB-Map algorithm parameters
    #
    # These are RTAB-Map native parameters, NOT ROS launch
    # parameters.
    #
    # Keeping them here makes SLAM tuning easy.
    # ============================================================

    rtabmap_args = (
        '--delete_db_on_start '
        '--Grid/CellSize 0.05 '
        '--Grid/RangeMin 0.3 '
        '--Grid/RangeMax 5.0 '
        '--Grid/MinClusterSize 5 '
        '--Grid/ClusterRadius 0.1 '
        '--Grid/RayTracing true '
        '--Grid/NormalSegmentation true '
        '--Grid/MinMapSize 1000 '
        '--Grid/MinObstacleHeight 0.05 '
        '--Grid/MaxObstacleHeight 2.0 '
        '--Grid/FlatObstacleDetected true '
        '--Grid/MaxGroundAngle 30.0 '
        '--Grid/3D false '
    )

    # ============================================================
    # ZED Odometry -> TF
    #
    # ZED publishes:
    #
    #   /zed/zed_node/odom
    #
    # This converts that Odometry message into:
    #
    #   odom -> base_link
    #
    # ============================================================

    odom_to_tf_node = Node(
        package='odom_to_tf_ros2',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',

        parameters=[{
            'odom_topic': '/zed/zed_node/odom',
            'frame_id': 'odom',
            'child_frame_id': 'base_link',
            'inverse_tf': False,
        }],
    )

    # ============================================================
    # RTAB-Map
    # ============================================================

    rtabmap_group = GroupAction([

        # --------------------------------------------------------
        # Apply Minibot topic remappings
        # --------------------------------------------------------

        *[
            SetRemap(
                src=source,
                dst=destination
            )
            for source, destination in remappings
        ],

        # --------------------------------------------------------
        # Include official RTAB-Map launch
        # --------------------------------------------------------

        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(
                rtabmap_launch
            ),

            launch_arguments={

                # =================================================
                # Robot / SLAM frames
                # =================================================

                'frame_id':
                    'base_link',

                'map_frame_id':
                    'map',

                # IMPORTANT:
                #
                # Empty means RTAB-Map consumes the Odometry
                # MESSAGE instead of obtaining odometry from TF.
                'odom_frame_id':
                    '',

                # =================================================
                # External odometry
                #
                # We use ZED SDK positional tracking.
                # =================================================

                'odom_topic':
                    '/zed/zed_node/odom',

                # DO NOT launch RTAB-Map RGB-D visual odometry.
                'visual_odometry':
                    'false',

                # DO NOT launch ICP odometry.
                'icp_odometry':
                    'false',

                # RTAB-Map odometry nodes don't own odom->base_link.
                'publish_tf_odom':
                    'false',

                # RTAB-Map SLAM owns map->odom.
                'publish_tf_map':
                    'true',

                # =================================================
                # RGB-D input
                # =================================================

                'subscribe_rgb':
                    'true',

                'subscribe_depth':
                    'true',

                'subscribe_rgbd':
                    'false',

                # =================================================
                # Synchronization
                # =================================================

                'approx_sync':
                    'true',

                # Reject RGB/depth combinations differing by
                # more than 10 ms.
                'approx_sync_max_interval':
                    '0.01',

                # Give TF time to become available for image
                # timestamps.
                'wait_for_transform':
                    '1.0',

                # =================================================
                # IMU
                # =================================================

                'imu_topic':
                    '/zed/zed_node/imu/data',

                # =================================================
                # RTAB-Map algorithm configuration
                # =================================================

                'rtabmap_args':
                    rtabmap_args,

                # =================================================
                # Visualization
                # =================================================

                'rviz':
                    'true',

                'rtabmap_viz':
                    'true',

            }.items()
        ),
    ])

    # ============================================================
    # Launch everything
    # ============================================================

    return LaunchDescription([

        odom_to_tf_node,

        rtabmap_group,

    ])