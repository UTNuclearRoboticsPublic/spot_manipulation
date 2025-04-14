from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from launch_ros.actions import SetRemap, SetParameter, PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from spot_description.get_accessories import get_accessories_from_env

def spot_config():
    xacro_args = get_accessories_from_env()
    xacro_args['kinematic_model'] = 'body_assist'
    moveit_config_builder = MoveItConfigsBuilder("spot", package_name="spot_moveit_config")
    moveit_config_builder.robot_description(mappings=xacro_args)
    moveit_config_builder.robot_description_semantic(mappings=xacro_args)
    return moveit_config_builder.to_moveit_configs()
    
def generate_launch_description():
    moveit_config = spot_config()
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
