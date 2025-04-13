from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap, Node
from spot_description.get_accessories import get_accessories_from_env

def generate_launch_description():
    spot_accessories = get_accessories_from_env()
    moveit_config_builder = MoveItConfigsBuilder("spot", package_name="spot_moveit_config")
    moveit_config_builder.robot_description(mappings=spot_accessories)
    moveit_config_builder.robot_description_semantic(mappings=spot_accessories)
    
    moveit_config = moveit_config_builder.to_moveit_configs()

    rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=f"{moveit_config.package_path}/config/moveit.rviz",
    )
    
    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description,
        {'moveit_namespace': 'spot_moveit'}
    ]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=rviz_parameters
    )

    return LaunchDescription([
        SetRemap(src='/spot_moveit/joint_states', dst='/spot_driver/joint_states'),
        SetRemap(src='/spot_moveit/arm_controller/follow_joint_trajectory', dst='/arm_controller/follow_joint_trajectory'),
        SetRemap(src='/spot_moveit/body_manipulation_controller/follow_joint_trajectory', dst='/body_manipulation_controller/follow_joint_trajectory'),
        rviz_config,
        rviz_node
    ])