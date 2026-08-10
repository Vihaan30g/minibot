from launch import LaunchDescription
from launch_ros.actions import Node
import os 
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    package_name = 'minibot_slam'
    package_share_dir = get_package_share_directory(package_name)

    # Sledgehammer fix for the visualizer
    optical_frame_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='optical_frame_alias',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0', 
            '--yaw', '-1.5708', '--pitch', '0', '--roll', '-1.5708', 
            '--frame-id', 'zed_camera_link', '--child-frame-id', 'zed_left_camera_frame_optical' # <--- EXACT MATCH NOW
        ]
    )
    # Bridge IMU to Camera
    imu_frame_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_frame_alias',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0', 
            '--yaw', '0', '--pitch', '0', '--roll', '0', 
            '--frame-id', 'zed_camera_link', '--child-frame-id', 'zed_imu_link'
        ]
    )

    # Bridge Rover Center (base_link) to Camera (The "Neck")
    base_to_camera_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera',
        arguments=[
            '--x', '0.2', '--y', '0', '--z', '0.5', 
            '--yaw', '0', '--pitch', '0', '--roll', '0', 
            '--frame-id', 'base_link', '--child-frame-id', 'zed_camera_link'
        ]
    )


    odom_to_tf_node = Node(
        package='odom_to_tf_ros2',
        executable='odom_to_tf',
        name='odom_to_tf_ros2',
        output='screen',
         parameters=[{
            'odom_topic':'/zed/zed_node/odom',
            'frame_id':'odom',
            'child_frame_id':'base_link',
            'inverse_tf':False,
        }],
    )

    

    # ---------------------------------------------------------
    # 2. Extended Kalman Filter (Sensor Fusion)
    # ---------------------------------------------------------
    # ekf_config = os.path.join(package_share_dir, 'config', 'ekf_config.yaml')

    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_config],
    #     remappings=[('odometry/filtered', '/odom_fused')]
    # )

    # ---------------------------------------------------------
    # 3. SLAM (RTAB-Map) Configuration
    # ---------------------------------------------------------
    rtabmap_parameters = {
        'frame_id':              'base_link', # Anchored to EKF base
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
    }

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
        parameters=[ 
            {'use_sim_time': True},
            rtabmap_parameters
        ],
        remappings=remappings,
        arguments=['-d']  
    )


    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[
            {'use_sim_time': True},   #DEBUG
            {
            'frame_id': 'base_link', 
            'subscribe_depth': True,
            'subscribe_odom_info': False,
            'approx_sync': True,
            'wait_for_transform': 0.5,  
        }],
        remappings=remappings
    )

    return LaunchDescription([
        optical_frame_fix,
        imu_frame_fix,
        base_to_camera_fix,
        #ekf_node,
        rtabmap_node,
        rtabmap_viz_node,
        odom_to_tf_node,
    ])
