from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import SetRemap
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("spot", package_name="spot_moveit_config").to_moveit_configs()
    moveit_config.move_group_capabilities["capabilities"] = ""
    return LaunchDescription([
        GroupAction(
            actions=[
                SetRemap(src='/joint_states', dst='/spot_driver/joint_states'),
                generate_move_group_launch(moveit_config)
            ]
        )
    ])
