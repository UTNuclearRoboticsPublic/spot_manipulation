import numpy as np
import rclpy.time
from google.protobuf import timestamp_pb2
from bosdyn.api import geometry_pb2, arm_command_pb2, robot_state_pb2
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .spot_manipulation_driver import SpotManipulationDriver
from spot_msgs.msg import ManipulatorState
from control_msgs.action import FollowJointTrajectory
from spot_msgs.action import ArmImpedanceCommand

def joint_trajectory_to_lists(msg: JointTrajectory):
    traj_point_positions = []
    traj_point_velocities = []
    timepoints = []

    # Order of joints
    joint_order = [
        "arm0_shoulder_yaw",
        "arm0_shoulder_pitch",
        "arm0_elbow_pitch",
        "arm0_elbow_roll",
        "arm0_wrist_pitch",
        "arm0_wrist_roll",
    ]

    # Reorder joint commands based on joint_order and put them into long lists of lists
    point: JointTrajectoryPoint
    for point in msg.points:
        pos_dict = {}
        vel_dict = {}

        for j in range(0, 6):
            name = msg.joint_names[j]
            pos_dict[name] = point.positions[j]
            vel_dict[name] = point.velocities[j]

        traj_point_positions.append(
            list(map(lambda joint_name: pos_dict[joint_name], joint_order))
        )
        traj_point_velocities.append(
            list(map(lambda joint_name: vel_dict[joint_name], joint_order))
        )
        timepoints.append(
            point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        )

    return traj_point_positions, traj_point_velocities, timepoints

def twist_to_vel_request(
        robot_time: timestamp_pb2.Timestamp, 
        msg: Twist, 
        linear_lims: "list[float]" = [-1e9, 1e9],
        angular_lims: "list[float]" = [-1e9, 1e9],
        robot_frame = "body") -> arm_command_pb2.ArmVelocityCommand.Request:

    # Enforce velocity limits
    linear_vel = np.clip(
        np.array([msg.linear.x, msg.linear.y, msg.linear.z]),
        linear_lims[0],
        linear_lims[1],
    )
    angular_vel = np.clip(
        np.array([msg.angular.x, msg.angular.y, msg.angular.z]),
        angular_lims[0],
        angular_lims[1],
    )

    # Assemble linear and angular components
    linear  = geometry_pb2.Vec3(x=linear_vel[0], y=linear_vel[1], z=linear_vel[2])
    angular = geometry_pb2.Vec3(x=angular_vel[0], y=angular_vel[1], z=angular_vel[2])

    # SDK requires cartesian velocity to be created separately
    end_effector_velocity = arm_command_pb2.ArmVelocityCommand.CartesianVelocity(
        frame_name=robot_frame, velocity_in_frame_name=linear
    )

    # TODO: Verify that this works
    # Velocity command will live for 0.1 seconds
    end_time = timestamp_pb2.Timestamp()
    end_time.CopyFrom(robot_time)
    end_time.FromMilliseconds(end_time.ToMilliseconds() + 100)

    arm_velocity_command = arm_command_pb2.ArmVelocityCommand.Request(
        cartesian_velocity=end_effector_velocity,
        angular_velocity_of_hand_rt_odom_in_hand=angular,
        end_time=end_time,
    )

    return arm_velocity_command

def get_joint_state_feedback(driver: SpotManipulationDriver) -> FollowJointTrajectory.Feedback:
    kinematic_state = driver.kinematic_state
    feedback = FollowJointTrajectory.Feedback()

    # Get robot time as local time
    local_time = driver._lease_manager.robotToLocalTime(kinematic_state.acquisition_timestamp)
    feedback.header.stamp.sec = local_time.seconds
    feedback.header.stamp.nanosec = local_time.nanos

    # Pack joint states into returnable variables
    for joint in kinematic_state.joint_states:
        if joint.name == "arm0.hr0":  # Ignore this joint
            continue
        feedback.joint_names.append(joint.name)
        feedback.actual.positions.append(joint.position.value)
        feedback.actual.velocities.append(joint.velocity.value)
        feedback.actual.effort.append(joint.load.value)
    return feedback

def get_arm_impedance_feedback(arm_impedance_command: arm_command_pb2.ArmImpedanceCommand,
                               driver: SpotManipulationDriver) -> ArmImpedanceCommand.Feedback:
    
    kinematic_state = driver.arm_state
    feedback = ArmImpedanceCommand.Feedback()

    # Get robot time as local time
    local_time = driver._lease_manager.robotToLocalTime(kinematic_state.acquisition_timestamp)
    feedback.header.stamp.sec = local_time.seconds
    feedback.header.stamp.nanosec = local_time.nanos

    feedback.status = arm_impedance_command.feedback.status
    feedback.commanded_wrench_from_stiffness_at_tool_in_desired_tool = arm_impedance_command.feedback.commanded_wrench_from_stiffness_at_tool_in_desired_tool
    feedback.commanded_wrench_from_damping_at_tool_in_desired_tool = arm_impedance_command.feedback.commanded_wrench_from_damping_at_tool_in_desired_tool
    feedback.commanded_wrench_from_feed_forward_at_tool_in_desired_tool = arm_impedance_command.feedback.commanded_wrench_from_feed_forward_at_tool_in_desired_tool
    feedback.total_commanded_wrench_at_tool_in_desired_tool = arm_impedance_command.feedback.total_commanded_wrench_at_tool_in_desired_tool
    feedback.total_measured_wrench_at_tool_in_desired_tool = arm_impedance_command.feedback.total_measured_wrench_at_tool_in_desired_tool

def manipulator_state_to_msg(manipulator_state: robot_state_pb2.ManipulatorState,
                           driver: SpotManipulationDriver) -> ManipulatorState:
    """Maps manipulator state data from robot state proto to ROS ManipulatorState message

    Args:
        manipulator_state: ManipulatorState proto
        spot_wrapper: A SpotWrapper object
    Returns:
        spot_msgs/ManipulatorState ROS message
    """
    stamp = driver.lease_manager.robotToLocalTime(driver.robot_time)
    ros_stamp = rclpy.time.Time(nanoseconds=stamp.ToNanoseconds())

    if manipulator_state is None:
        return ManipulatorState()
    manipulator_state_msg = ManipulatorState()
    manipulator_state_msg.gripper_open_percentage = manipulator_state.gripper_open_percentage
    manipulator_state_msg.is_gripper_holding_item = manipulator_state.is_gripper_holding_item
    manipulator_state_msg.estimated_end_effector_force_in_hand.header.frame_id = "arm0_hand"
    manipulator_state_msg.estimated_end_effector_force_in_hand.header.stamp = ros_stamp.to_msg()
    manipulator_state_msg.estimated_end_effector_force_in_hand.wrench.force.x = manipulator_state.estimated_end_effector_force_in_hand.x
    manipulator_state_msg.estimated_end_effector_force_in_hand.wrench.force.y = manipulator_state.estimated_end_effector_force_in_hand.y
    manipulator_state_msg.estimated_end_effector_force_in_hand.wrench.force.z = manipulator_state.estimated_end_effector_force_in_hand.z
    manipulator_state_msg.stow_state = manipulator_state.stow_state
    # manipulator_state_msg.velocity_of_hand_in_vision = manipulator_state.velocity_of_hand_in_vision
    # manipulator_state_msg.velocity_of_hand_in_odom = manipulator_state.velocity_of_hand_in_odom
    manipulator_state_msg.carry_state = manipulator_state.carry_state
    return manipulator_state_msg