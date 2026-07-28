from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    package_name = "minibot_description"

    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        "urdf",
        "minibot.urdf"
    )

    with open(urdf_file, "r") as infp:
        robot_description = infp.read()

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

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen"
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen"
        ),

    ])