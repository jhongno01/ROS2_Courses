from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lidar_tilt_safe_drive",
            executable="lidar_tilt_safe_drive",
            name="lidar_tilt_safe_drive",
            output="screen",
        ),
    ])
