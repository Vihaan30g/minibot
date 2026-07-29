from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    package_name = "minibot_description"

    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        "urdf",
        "slam_minimal.urdf"
    )

    with open(urdf_file, "r") as infp:
        robot_description = infp.read()







    base_to_camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_zed_camera_link',
        output='screen',
        arguments=[
            '--x', '0.19', '--y', '0', '--z', '0.426',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'zed_camera_link',   # must match camera_name used below
        ],
    )

    odom_to_tf_node = Node(
        package='odom_to_tf_ros2',
        executable='odom_to_tf',
        name='odom_to_tf_ros2',
        output='screen',
        parameters=[{
            'odom_topic': '/zed/zed_node/odom',
            'frame_id': 'odom',
            'child_frame_id': 'base_link',
            'inverse_tf': False,
        }],
    )

    return LaunchDescription([

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "use_sim_time": False
                }
            ]
        ),
        base_to_camera_tf_node,
        odom_to_tf_node,
        

    ])