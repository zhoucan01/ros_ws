import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic", default="/wheel_raw")
    output_topic = LaunchConfiguration("output_topic", default="/wheel_odom_transformed")
    output_frame = LaunchConfiguration("output_frame", default="base_link")
    enable_yaw = LaunchConfiguration("enable_yaw_correction", default="false")

    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="/wheel_raw"),
        DeclareLaunchArgument("output_topic", default_value="/wheel_odom_transformed"),
        DeclareLaunchArgument("output_frame", default_value="base_link"),
        DeclareLaunchArgument("enable_yaw_correction", default_value="false"),
        Node(
            package="wheel_observer",
            executable="wheel_observer_node",
            name="wheel_observer",
            output="screen",
            parameters=[{
                "input_topic": input_topic,
                "output_topic": output_topic,
                "output_frame_id": output_frame,
                "enable_yaw_correction": enable_yaw,
            }],
        ),
    ])
