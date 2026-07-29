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

    odom_to_tf_node = Node(
        package='odom_to_tf_ros2',
        executable='odom_to_tf',
        name='odom_to_tf_ros2',
        output='screen',
        parameters=[{
            'odom_topic':            '/zed/zed_node/odom',
            'frame_id':              'odom',
            'child_frame_id':        'base_link',
            'inverse_tf':            False,
        }],
    )

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
        arguments=['-d'],
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
            'wait_for_transform': 2.0,  
        }],
        remappings=remappings
    )

    return LaunchDescription([
        odom_to_tf_node,
        rtabmap_node,
        rtabmap_viz_node,  
    ])