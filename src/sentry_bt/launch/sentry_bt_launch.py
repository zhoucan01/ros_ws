from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("sentry_bt")
    default_params = os.path.join(pkg_share, "config", "sentry_bt_params.yaml")

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="red_standard_robot1",
                description="ROS namespace for sentry_bt",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to sentry_bt params YAML",
            ),
            Node(
                package="sentry_bt",
                executable="sentry_bt_node",
                namespace=namespace,
                parameters=[params_file],
                output="screen",
            ),
        ]
    )
