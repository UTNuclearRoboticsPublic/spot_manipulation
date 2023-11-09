from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params_file = PathJoinSubstitution([FindPackageShare('spot_manipulation_driver'), 'config', 'arm_teleop_joy.yaml'])

    return LaunchDescription([
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
    ])
