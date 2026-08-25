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
    #    Publishes /wheel/odom (topic only). Make sure the node in
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
    # ---------------------------------------------------------
    rtabmap_parameters = [{
        'frame_id':              'base_link',
        'map_frame_id':          'map',
        'odom_frame_id':         'odom',

        'subscribe_stereo':      False,
        'subscribe_depth':       True,
        'subscribe_odom_info':   False,
        'approx_sync':           True,

        'wait_for_transform':    2.0,
        'wait_imu_to_init':      True,

        'Vis/FeatureType':       '8',
        'Kp/DetectorStrategy':   '8',
        'Vis/MaxFeatures':       '1000',
        'Vis/MinInliers':        '25',
        'Rtabmap/LoopThr':       '0.11',
        'Optimizer/Robust':      'true',
        'Rtabmap/TimeThr':       '700',

        'Grid/3D':               'true',
        'Grid/CellSize':         '0.05',
        'Grid/RangeMax':         '10.0',
        'Grid/RayTracing':       'true',
        'Landmark/Enabled':      'true',
    }]

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
        parameters=rtabmap_parameters,
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