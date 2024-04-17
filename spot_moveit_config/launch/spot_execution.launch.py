import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF
from moveit_configs_utils import MoveItConfigsBuilder

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

    moveit_config = MoveItConfigsBuilder("spot", package_name="spot_moveit_config").to_moveit_configs()

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    ld.add_action(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=move_group_params,
            remappings=[
                ("/joint_states", "/spot_driver/joint_states")
            ]
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