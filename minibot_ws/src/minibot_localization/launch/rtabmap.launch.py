from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    rtabmap_launch = os.path.join(
        get_package_share_directory('rtabmap_launch'),
        'launch',
        'rtabmap.launch.py'
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch),

            launch_arguments={

                # RGB image
                'rgb_topic':
                    '/zed/zed_node/rgb/color/rect/image',

                # Depth image
                'depth_topic':
                    '/zed/zed_node/depth/depth_registered',

                # RGB camera info
                'camera_info_topic':
                    '/zed/zed_node/rgb/color/rect/camera_info',

                # ZED visual odometry
                'odom_topic':
                    '/zed/zed_node/odom',

                # Robot base frame
                'frame_id':
                    'base_link',

                'subscribe_rgbd':
                    'false',

                'approx_sync':
                    'true',

                'rviz':
                    'true',

                'rtabmap_args':
                    '--delete_db_on_start'

            }.items()

        )

    ])