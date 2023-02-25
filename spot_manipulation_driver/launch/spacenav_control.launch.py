from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="spacenav_node",
                executable="spacenav_node",
                name="spacenav_node",
                output="screen",
            )
        ]
    )
