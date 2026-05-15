import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory("turtlebot_autonomous_gap_drive")
    default_params_file = os.path.join(
        package_share_dir,
        "config",
        "slow_finish_first.yaml",
    )

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Path to the gap drive parameter YAML file.",
        ),
        Node(
            package="turtlebot_autonomous_gap_drive",
            executable="gap_drive_node",
            name="turtlebot_autonomous_gap_drive",
            output="screen",
            parameters=[params_file],
        ),
    ])
