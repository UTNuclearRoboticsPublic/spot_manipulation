from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import SetRemap, SetParameter, PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from spot_description.get_accessories import get_accessories_from_env

def generate_launch_description():
    spot_accessories = get_accessories_from_env()
    moveit_config_builder = MoveItConfigsBuilder("spot", package_name="spot_moveit_config")
    moveit_config_builder.robot_description(mappings=spot_accessories)
    moveit_config_builder.robot_description_semantic(mappings=spot_accessories)
    
    moveit_config = moveit_config_builder.to_moveit_configs()
    moveit_config.move_group_capabilities["capabilities"] = ""


    return LaunchDescription([
        GroupAction(
            actions=[
                PushRosNamespace("spot_moveit"),
                SetRemap(src='/spot_moveit/joint_states', dst='/spot_driver/joint_states'),
                SetRemap(src='/spot_moveit/robot_description', dst='/spot_driver/robot_description'),
                SetParameter(name="octomap_resolution", value=0.03),
                generate_move_group_launch(moveit_config)
            ]
        )
    ])
