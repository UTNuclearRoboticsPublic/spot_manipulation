import numpy as np
from bosdyn.api import arm_command_pb2, geometry_pb2, robot_state_pb2
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist, TransformStamped
from google.protobuf import timestamp_pb2
from spot_msgs.msg import ManipulatorState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.spatial.transform import Rotation as R

import rclpy.time

from .spot_manipulation_driver import SpotManipulationDriver


def wbc_joint_trajectory_to_lists(msg: JointTrajectory, T_o2f, yaw_o2f):
    traj_point_positions = []
    traj_point_velocities = []
    timepoints = []

    # Order of joints
    joint_order = [
        "body_x",
        "body_y",
        "body_or",
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
        # vel_dict = {}

        for j in range(0, len(joint_order)):
            name = msg.joint_names[j]
            pos_dict[name] = point.positions[j]
            # vel_dict[name] = point.velocities[j]

        traj_point_positions.append(
            list(map(lambda joint_name: pos_dict[joint_name], joint_order))
        )

        # Transform the body joints from body to odom frame
        [x,y,theta] = traj_point_positions[-1][:3]
        traj_point_positions[-1][:3] = transform_planar_point_and_or(x, y, theta, T_o2f, yaw_o2f)

        # traj_point_velocities.append(
        #     list(map(lambda joint_name: vel_dict[joint_name], joint_order))
        # )
        timepoints.append(
            point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        )

    return traj_point_positions, traj_point_velocities, timepoints

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
        # vel_dict = {}

        for j in range(0, 6):
            name = msg.joint_names[j]
            pos_dict[name] = point.positions[j]
            # vel_dict[name] = point.velocities[j]

        traj_point_positions.append(
            list(map(lambda joint_name: pos_dict[joint_name], joint_order))
        )
        # traj_point_velocities.append(
        #     list(map(lambda joint_name: vel_dict[joint_name], joint_order))
        # )
        timepoints.append(
            point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        )

    return traj_point_positions, traj_point_velocities, timepoints


def twist_to_vel_request(
    robot_time: timestamp_pb2.Timestamp,
    msg: Twist,
    linear_lims: "list[float]" = [-1e9, 1e9],
    angular_lims: "list[float]" = [-1e9, 1e9],
    robot_frame="body",
) -> arm_command_pb2.ArmVelocityCommand.Request:

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
    linear = geometry_pb2.Vec3(x=linear_vel[0], y=linear_vel[1], z=linear_vel[2])
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


def get_joint_state_feedback(
    driver: SpotManipulationDriver
) -> FollowJointTrajectory.Feedback:
    kinematic_state = driver.kinematic_state
    feedback = FollowJointTrajectory.Feedback()

    # Get robot time as local time
    local_time = driver._lease_manager.robotToLocalTime(
        kinematic_state.acquisition_timestamp
    )
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


def manipulator_state_to_msg(
    manipulator_state: robot_state_pb2.ManipulatorState, driver: SpotManipulationDriver
) -> ManipulatorState:
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
    manipulator_state_msg.gripper_open_percentage = (
        manipulator_state.gripper_open_percentage
    )
    manipulator_state_msg.is_gripper_holding_item = (
        manipulator_state.is_gripper_holding_item
    )
    manipulator_state_msg.estimated_end_effector_force_in_hand.header.frame_id = (
        "arm0_hand"
    )
    manipulator_state_msg.estimated_end_effector_force_in_hand.header.stamp = (
        ros_stamp.to_msg()
    )
    manipulator_state_msg.estimated_end_effector_force_in_hand.wrench.force.x = (
        manipulator_state.estimated_end_effector_force_in_hand.x
    )
    manipulator_state_msg.estimated_end_effector_force_in_hand.wrench.force.y = (
        manipulator_state.estimated_end_effector_force_in_hand.y
    )
    manipulator_state_msg.estimated_end_effector_force_in_hand.wrench.force.z = (
        manipulator_state.estimated_end_effector_force_in_hand.z
    )
    manipulator_state_msg.stow_state = manipulator_state.stow_state
    # manipulator_state_msg.velocity_of_hand_in_vision = manipulator_state.velocity_of_hand_in_vision
    # manipulator_state_msg.velocity_of_hand_in_odom = manipulator_state.velocity_of_hand_in_odom
    manipulator_state_msg.carry_state = manipulator_state.carry_state
    return manipulator_state_msg


def convert_transformstamped_to_matrix(transform_stamped):
    # Extract quaternion and translation components
    quat_x = transform_stamped.transform.rotation.x
    quat_y = transform_stamped.transform.rotation.y
    quat_z = transform_stamped.transform.rotation.z
    quat_w = transform_stamped.transform.rotation.w
    x = transform_stamped.transform.translation.x
    y = transform_stamped.transform.translation.y
    z = transform_stamped.transform.translation.z

    # Construct HTM representation
    rot = R.from_quat([quat_x, quat_y, quat_z, quat_w])
    euler_o2f = rot.as_euler('zxy')
    T_o2f = np.eye(4)
    T_o2f[:3, :3] = rot.as_matrix()  # Assign rotation matrix
    T_o2f[:3, 3] = [x, y, z]       # Assign translation vector

    return T_o2f, euler_o2f[0]

def transform_planar_point_and_or(x, y, yaw_f2b, T_o2f, yaw_o2f):


    # Create HTM representation base_footprint to body transformation
    T_f2b = np.eye(4)

    # Assign rotation to the HTM
    R_f2b = R.from_euler('z', yaw_f2b)
    T_f2b[:3, :3] = R_f2b.as_matrix()

    # Assign translation to the HTM
    p_b = [x, y, 0]  # z 0 since footprint
    T_f2b[:3, 3] = p_b

    # Compute HTM representing body in odom
    T_o2b = T_o2f @ T_f2b

    p_o = [T_o2b[0, 3], T_o2b[1, 3], yaw_o2f + yaw_f2b]  # Extract transformed x, y, and updated yaw

    return p_o
