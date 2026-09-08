from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("algo_master")
    default_params = os.path.join(pkg_share, "config", "algo_master_params.yaml")

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="red_standard_robot1",
                description="ROS namespace for algo_master",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to algo_master params YAML",
            ),
            Node(
                package="algo_master",
                executable="algo_master_node",
                name="algo_master",
                namespace=namespace,
                parameters=[params_file],
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
                output="screen",
            ),
        ]
    )
