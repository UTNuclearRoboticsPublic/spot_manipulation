from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import SetRemap, SetParameter, PushRosNamespace, Node
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from spot_description.get_accessories import get_accessories_from_env


def launch_setup(context, *args, **kwargs):
    # The MoveIt config is built here (inside an OpaqueFunction) rather than in
    # generate_launch_description so the octomap arg is resolved before the
    # config is finalized.
    xacro_args = get_accessories_from_env()
    xacro_args['kinematic_model'] = LaunchConfiguration('kinematic_model')
    moveit_config_builder = MoveItConfigsBuilder("spot", package_name="spot_moveit_config")
    moveit_config_builder.robot_description(mappings=xacro_args)
    moveit_config_builder.robot_description_semantic(mappings=xacro_args)
    moveit_config = moveit_config_builder.to_moveit_configs()

    # octomap:=false empties the sensors_3d updaters, so move_group perceives no
    # octomap. Calibration uses this: near-plate goals otherwise land inside the
    # perceived-plate voxels (goal-state sampling failure). The registered plate
    # PLANE still provides plate collision. Discovery leaves octomap true.
    if LaunchConfiguration('octomap').perform(context).strip().lower() in ('false', '0', 'no'):
        moveit_config.sensors_3d = {}

    stable_motion_server = Node(
        package='spot_moveit_config',
        executable='stable_arm_motion_server'
    )

    return [
        GroupAction(
            actions=[
                PushRosNamespace("spot_moveit"),
                SetRemap(src='/spot_moveit/joint_states', dst='/spot_driver/joint_states'),
                SetRemap(src='/spot_moveit/robot_description', dst='/spot_driver/robot_description'),
                SetParameter(name='use_sim_time',
                             value=ParameterValue(LaunchConfiguration('use_sim_time'),
                                                  value_type=bool)),
                SetParameter(name="octomap_resolution", value=0.075),
                SetParameter(name="octomap_frame", value="spot_nav/map"),
                generate_move_group_launch(moveit_config),
                stable_motion_server
            ]
        )
    ]


def generate_launch_description():

    # Launch args
    launch_args = [
        DeclareLaunchArgument('kinematic_model',
                            description='The kinematic model to use for the Spot description',
                            choices=['none', 'body_assist', 'mobile_manipulation'],
                            default_value='none'),
        DeclareLaunchArgument('use_sim_time',
                            description='Drive move_group off the /clock topic instead of the '
                                        'system clock. Required under Gazebo, where joint states '
                                        'are sim-time stamped. Leave false on hardware.',
                            default_value='false'),
        DeclareLaunchArgument('octomap',
                            description='false empties the sensors_3d updaters so move_group builds '
                                        'no octomap (calibration; the plate PLANE still collides). '
                                        'Leave true for discovery.',
                            default_value='true'),
    ]

    return LaunchDescription([
        *launch_args,
        OpaqueFunction(function=launch_setup)
    ])