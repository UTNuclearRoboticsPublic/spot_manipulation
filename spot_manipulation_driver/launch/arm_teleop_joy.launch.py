from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_arguments = [

    ]

    return LaunchDescription([
        *launch_arguments,
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name='spot_manipulator_teleop_joy',
            parameters=[
                PathJoinSubstitution([FindPackageShare('spot_manipulation_driver'), 'config', 'arm_teleop_joy.yaml'])
            ],
            remappings=[
                ('/cmd_vel', '/follow_joint_trajectory_node/cmd_vel')
            ]
        )
    ])
