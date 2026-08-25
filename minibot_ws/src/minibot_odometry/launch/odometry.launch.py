from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    wheel_odometry_node = Node(
        package='minibot_odometry',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        output='screen',
    )

    return LaunchDescription([
        wheel_odometry_node,
    ])