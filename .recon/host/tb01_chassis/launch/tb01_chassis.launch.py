import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("tb01_chassis")
    default_params = os.path.join(package_share, "config", "tb01_chassis.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="tb01_chassis 参数文件路径",
            ),
            Node(
                package="tb01_chassis",
                executable="tb01_chassis",
                name="scrubber_damiao_chassis",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
