import os 
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package share directory
    package_name = 'minibot_localization'
    package_share_dir = get_package_share_directory(package_name)
    
    # Path to EKF config file
    rtabmap_config_path = os.path.join(package_share_dir, 'config', 'rtabmap_config.yaml')

    # ---------------------------------------------------------
    # 1. Transforms (CRITICAL: Using strict Humble syntax to prevent buffer crash)
    # ---------------------------------------------------------
    
    # ---------------------------------------------------------
    # 2. Extended Kalman Filter (Sensor Fusion)
    # ---------------------------------------------------------


    odom_to_tf_node = Node(
        package='odom_to_tf_ros2',
        executable='odom_to_tf',
        name='odom_to_tf_ros2',
        output='screen',
        parameters=[{
            'odom_topic':            "/zed/zed_node/odom",
            'frame_id':              "odom",
            'child_frame_id':        "base_link",
            'inverse_tf':            False,
        }],
    )
    # ---------------------------------------------------------
    # 4. Topic Remappings (Shared by SLAM and VIZ)
    # ---------------------------------------------------------
    remappings = [
        ('rgb/image',       '/zed/zed_node/rgb/color/rect/image'),
        ('rgb/camera_info', '/zed/zed_node/rgb/color/rect/camera_info'),
        ('depth/image',     '/zed/zed_node/depth/depth_registered'),
        ('odom',            '/zed/zed_node/odom'),  # RTAB-Map listens to EKF output
        ('imu',             '/zed/zed_node/imu/data'),
    ]

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[rtabmap_config_path],
        remappings=remappings,
        arguments=['-d']  
    )

    # ---------------------------------------------------------
    # 5. ArUco & Visualizer
    # ---------------------------------------------------------
    # aruco_node = Node(
    #     package='aruco_ros',
    #     executable='marker_publisher',
    #     name='aruco_marker_publisher',
    #     output='screen',
    #     parameters=[{
    #         'image_is_rectified': True,
    #         'marker_size':        0.30,  
    #         'dictionary':         0,     
    #         'reference_frame':    'zed_left_camera_optical_frame',
    #         'camera_frame':       'zed_left_camera_optical_frame',
    #     }],
    #     remappings=[
    #         ('/camera_info', '/zed/zed_node/rgb/color/rect/camera_info'),
    #         ('/image',       '/zed/zed_node/rgb/color/rect/image'),
    #     ]
    # )

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[{
            'frame_id': 'base_link', 
            'subscribe_depth': True,
            'subscribe_odom_info': False,
            'approx_sync': True,
            'wait_for_transform': 2.0,  
        }],
        remappings=remappings
    )

    return LaunchDescription([
        odom_to_tf_node,
        rtabmap_node,
        # aruco_node,
        rtabmap_viz_node,  
    ])