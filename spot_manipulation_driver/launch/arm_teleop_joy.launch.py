from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def configure_teleop_nodes(ctx, *args, **kwargs):
    suffix = LaunchConfiguration('controller_configuration').perform(ctx).lower()
    params_file = PathJoinSubstitution([FindPackageShare('spot_manipulation_driver'), 'config', f'arm_teleop_{suffix}.yaml'])

    return [
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name='spot_manipulator_teleop_cartesian',
            parameters=[
                params_file
            ],
            remappings=[
                ('/cmd_vel', '/spot_manipulation_driver/cmd_vel')
            ]
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name='spot_manipulator_teleop_angular',
            parameters=[
                params_file
            ],
            remappings=[
                ('/cmd_vel', '/spot_manipulation_driver/cmd_vel')
            ]
        )
    ]

def generate_launch_description():
    controller_config = DeclareLaunchArgument('controller_configuration',
        description='Name of the controller configuration to use for teleoperation',
        choices=['Logitech', 'Dualsense5'],
        default_value='Logitech'
    )

    return LaunchDescription([
        controller_config,
        OpaqueFunction(function = configure_teleop_nodes)
    ])
