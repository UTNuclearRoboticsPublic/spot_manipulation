from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ld = LaunchDescription()

    moveit_config = MoveItConfigsBuilder("spot").to_moveit_configs()

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            )
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
            ),
        )
    )

    return ld
