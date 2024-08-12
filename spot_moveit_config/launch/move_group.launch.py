from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import SetRemap, SetParameter, PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("spot", package_name="spot_moveit_config").to_moveit_configs()
    moveit_config.move_group_capabilities["capabilities"] = ""
    return LaunchDescription([
        GroupAction(
            actions=[
                SetRemap(src='/spot_moveit/joint_states', dst='/spot_driver/joint_states'),
                SetParameter(name="octomap_resolution", value=0.03),
                PushRosNamespace("spot_moveit"),
                generate_move_group_launch(moveit_config)
            ]
        )
    ])
