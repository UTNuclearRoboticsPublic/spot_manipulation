from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap, SetParameter
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from spot_description.get_accessories import get_accessories_from_env    

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument('rviz', default_value='True'),
    ]

    xacro_args = get_accessories_from_env()
    xacro_args['kinematic_model'] = 'none'
    moveit_config_builder = MoveItConfigsBuilder('spot', package_name='spot_moveit_config')
    moveit_config_builder.robot_description(mappings=xacro_args)
    moveit_config_builder.robot_description_semantic(mappings=xacro_args)
    moveit_config = moveit_config_builder.to_moveit_configs()

    package_share = FindPackageShare('spot_moveit_config')
    config_dir = PathJoinSubstitution([package_share, 'config'])

    # ros2_control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            PathJoinSubstitution([config_dir, 'controllers.yaml']),
        ],
        remappings=[
            ('/controller_manager/robot_description', '/spot_driver/robot_description'),
            ('/joint_states', 'spot_driver/joint_states')
        ],
        output='screen',
    )

    ros2_control_remappings = [
        ('/controller_manager/configure_controller', '/spot_moveit/controller_manager/configure_controller'),
        ('/controller_manager/list_controllers', '/spot_moveit/controller_manager/list_controllers'),
        ('/controller_manager/list_controller_types', '/spot_moveit/controller_manager/list_controller_types'),
        ('/controller_manager/list_hardware_components', '/spot_moveit/controller_manager/list_hardware_components'),
        ('/controller_manager/list_hardware_interfaces', '/spot_moveit/controller_manager/list_hardware_interfaces'),
        ('/controller_manager/load_controller', '/spot_moveit/controller_manager/load_controller'),
        ('/controller_manager/reload_controller_libraries', '/spot_moveit/controller_manager/reload_controller_libraries'),
        ('/controller_manager/set_hardware_component_state', '/spot_moveit/controller_manager/set_hardware_component_state'),
        ('/controller_manager/set_hardware_component_state', '/spot_moveit/controller_manager/set_hardware_component_state'),
        ('/controller_manager/switch_controller', '/spot_moveit/controller_manager/switch_controller'),
        ('/controller_manager/unload_controller', '/spot_moveit/controller_manager/unload_controller'),
    ]

    # Spawn controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        remappings=ros2_control_remappings
    )

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        remappings=ros2_control_remappings
    )

    # Robot state publisher (uses the same robot_description)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[moveit_config.robot_description],
        output='screen',
    )

    # MoveGroup
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "move_group.launch.py"])
        )
    )

    # RViz with MoveIt config
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "moveit_rviz.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription(
        [
            *launch_args,
            GroupAction(actions=[
                SetParameter(name="publish_planning_scene_hz", value="30.0"),
                move_group_launch,
            ]),
            rviz_launch,
            GroupAction(
                actions=[
                    PushRosNamespace('spot_moveit'),
                    SetRemap(src='/joint_states', dst='/spot_driver/joint_states'),
                    SetRemap(src='/spot_moveit/joint_states', dst='/spot_driver/joint_states'),
                    SetRemap(src='/spot_moveit/robot_description', dst='/spot_driver/robot_description'),
                    SetRemap(src='/spot_moveit/controller_manager/robot_description', dst='/spot_driver/robot_description'),
                    ros2_control_node,
                    rsp,
                    joint_state_broadcaster_spawner,
                    arm_spawner,
                ]
            )
        ]
    )
