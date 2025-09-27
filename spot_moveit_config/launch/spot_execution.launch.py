import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    package_share = get_package_share_directory('spot_moveit_config')

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "move_group.launch.py")
        )
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "moveit_rviz.launch.py")
        )
    )

    return LaunchDescription([
        move_group_launch,
        rviz_launch
    ])