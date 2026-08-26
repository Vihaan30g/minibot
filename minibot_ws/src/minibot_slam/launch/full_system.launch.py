from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Full real-robot bring-up: camera + wheel odometry + EKF + rtabmap,
    all on wall-clock time (use_sim_time defaults to false everywhere).

    Use this once you're done tuning in tuning_replay.launch.py and are
    ready to run the robot for real. It simply brings up zed.launch.py
    (camera + static tf) and localization.launch.py (wheel odom + ekf +
    rtabmap) together -- the same components you used unmapped in
    record_bringup.launch.py, plus the ekf/rtabmap layer on top.
    """
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('minibot_slam'), '/launch/zed.launch.py']
        )
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('minibot_slam'), '/launch/localization.launch.py']
        )
    )

    return LaunchDescription([
        zed_launch,
        localization_launch,
    ])
