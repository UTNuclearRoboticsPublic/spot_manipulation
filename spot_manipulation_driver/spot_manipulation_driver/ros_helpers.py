import numpy as np
import rclpy.duration
from tf2_ros import Buffer
from typing import Tuple, List
from bosdyn.api import arm_command_pb2, geometry_pb2, robot_state_pb2, image_pb2, trajectory_pb2
from bosdyn.util import seconds_to_duration
from bosdyn.client.math_helpers import SE3Pose, Quat
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, HAND_FRAME_NAME, get_a_tform_b
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist, TwistStamped, WrenchStamped, Wrench
from google.protobuf import timestamp_pb2
from spot_msgs.msg import ManipulatorState
from spot_msgs.action import ArmCartesianCommand
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rclpy.time

from .spot_manipulation_driver import SpotManipulationDriver
from spot_driver import ros_helpers
from spot_driver.type_hint_helpers import *

joint_order_9DoF = [
    "body_height_joint",
    "body_yaw_joint",
    "body_pitch_joint",
    "body_roll_joint",
    "arm0_shoulder_yaw",
    "arm0_shoulder_pitch",
    "arm0_elbow_pitch",
    "arm0_elbow_roll",
    "arm0_wrist_pitch",
    "arm0_wrist_roll",
]

def MsgToWrench(wrench_msg: Wrench) -> WrenchProto:
    """Convert geometry_msgs.msg.Wrench to geometry_pb2.Wrench"""
    return geometry_pb2.Wrench(
        force=ros_helpers.MsgToVec3(wrench_msg.force),
        torque=ros_helpers.MsgToVec3(wrench_msg.torque)
    )

def WrenchToMsg(wrench_proto: WrenchProto) -> Wrench:
    """Convert geometry_pb2.Wrench to geometry_msgs.msg.Wrench"""
    return Wrench(
        force=ros_helpers.Vec3ToMsg(wrench_proto.force),
        torque=ros_helpers.Vec3ToMsg(wrench_proto.torque)
    )

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


def get_body_manipulation_trajectories(msg: JointTrajectory) -> Tuple[List[SE3Pose], List[List[float]], List[float]]:
    """
    Maps a given ROS joint trajectory to two trajectories: one for the body and one for the arm.
    The body trajectory contains a sequence of SE3Pose math_helpers messages 
    The arm trajectory contains a sequence of joint positions

    Args:
        msg: The input 9DoF joint trajectory planned for the robot arm with body assist
    Returns:
        List of math_helpers.SE3Pose messages
        List of joint state lists
        List of timestamps for each point of each trajectory
    """

    if len(msg.joint_names) != len(joint_order_9DoF):
        raise Exception(f"Invalid joint trajectory passed to body manipulation executor. Got {len(msg.joint_names)} points, expected {len(joint_order_9DoF)}")

    timestamps : List[float]       = []
    arm_points : List[List[float]] = []
    body_points: List[List[float]] = []
    body_poses : List[SE3Pose]     = []

    # Get the arm portion of the trajectory
    point: JointTrajectoryPoint # to get intellisense type hints
    for point in msg.points:
        body_point = [0.0, 0.0, 0.0, 0.0]
        arm_point  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        for joint_idx, joint_name in enumerate(joint_order_9DoF):
            msg_idx = msg.joint_names.index(joint_name)
            if joint_name.startswith("arm0"): 
                arm_point[joint_idx-4] = point.positions[msg_idx]
            else:
                body_point[joint_idx] = point.positions[msg_idx]

        arm_points.append(arm_point)
        body_points.append(body_point)
        timestamps.append(point.time_from_start.sec + point.time_from_start.nanosec * 1e-9)

    # Get the body portion of the trajectory
    for translation, yaw, pitch, roll in body_points:
        T1 = SE3Pose(0, 0, translation-0.52, Quat()) # 0.52 meters is the nominal standing height of spot
        T2 = SE3Pose(0, 0, 0, Quat.from_yaw(yaw))
        T3 = SE3Pose(0, 0, 0, Quat.from_pitch(pitch))
        T4 = SE3Pose(0, 0, 0, Quat.from_roll(roll))
        body_poses.append(T4*T3*T2*T1)

    return body_poses, arm_points, timestamps

def twist_to_vel_request(
    robot_time: timestamp_pb2.Timestamp,
    msg: Twist,
    linear_lims: "list[float]" = [-1e9, 1e9],
    angular_lims: "list[float]" = [-1e9, 1e9],
    robot_frame="body",
) -> ArmVelocityCommandProto:

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

def manipulator_state_to_wrench(
    manipulator_state: ManipulatorStateProto, driver: SpotManipulationDriver
) -> WrenchStamped:
    """Converts a manipultor state estimated force in hand to a WrenchStamped message

    Args:
        manipulator_state: ManipulatorState proto
        driver: A SpotManipulationDriver object to access robot time and state
    Returns:
        geometry_msgs/msg/WrenchStamped ROS message
    """
    stamp = driver.lease_manager.robotToLocalTime(driver.robot_time)
    wrench = WrenchStamped()
    wrench.header.frame_id = HAND_FRAME_NAME
    wrench.header.stamp = rclpy.time.Time(nanoseconds=stamp.ToNanoseconds()).to_msg()
    wrench.wrench.force = ros_helpers.Vec3ToMsg(manipulator_state.estimated_end_effector_force_in_hand)
    return wrench

def manipulator_state_to_twist(
    manipulator_state: ManipulatorStateProto, driver: SpotManipulationDriver
) -> TwistStamped:
    """Converts a manipultor state velocity in odom to a TwistStamped message

    Args:
        manipulator_state: ManipulatorState proto
        driver: A SpotManipulationDriver object to access robot time and state
    Returns:
        geometry_msgs/msg/TwistStamped ROS message
    """
    stamp = driver.lease_manager.robotToLocalTime(driver.robot_time)
    twist = TwistStamped()
    twist.header.frame_id = HAND_FRAME_NAME
    twist.header.stamp = rclpy.time.Time(nanoseconds=stamp.ToNanoseconds()).to_msg()
    hand_tform_odom = get_a_tform_b(driver.kinematic_state.transforms_snapshot, HAND_FRAME_NAME, ODOM_FRAME_NAME)
    twist.twist.linear = ros_helpers.Vec3ToMsg(hand_tform_odom.rotation.transform_vec3(manipulator_state.velocity_of_hand_in_odom.linear))
    twist.twist.angular = ros_helpers.Vec3ToMsg(hand_tform_odom.rotation.transform_vec3(manipulator_state.velocity_of_hand_in_odom.angular))
    return twist

def manipulator_state_to_msg(
    manipulator_state: ManipulatorStateProto, driver: SpotManipulationDriver
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
    manipulator_state_msg.gripper_open_percentage = manipulator_state.gripper_open_percentage
    manipulator_state_msg.is_gripper_holding_item = manipulator_state.is_gripper_holding_item

    manipulator_state_msg.stow_state  = manipulator_state.stow_state
    manipulator_state_msg.carry_state = manipulator_state.carry_state
    
    manipulator_state_msg.estimated_end_effector_force_in_hand.header.frame_id = "arm0_hand"
    manipulator_state_msg.estimated_end_effector_force_in_hand.header.stamp = ros_stamp.to_msg()
    manipulator_state_msg.estimated_end_effector_force_in_hand.wrench.force = ros_helpers.Vec3ToMsg(manipulator_state.estimated_end_effector_force_in_hand)

    # manipulator_state_msg.velocity_of_hand_in_vision = manipulator_state.velocity_of_hand_in_vision
    hand_tform_odom = get_a_tform_b(driver.kinematic_state.transforms_snapshot, HAND_FRAME_NAME, ODOM_FRAME_NAME)
    manipulator_state_msg.velocity_of_hand_in_odom.twist.linear = ros_helpers.Vec3ToMsg(hand_tform_odom.rotation.transform_vec3(manipulator_state.velocity_of_hand_in_odom.linear))
    manipulator_state_msg.velocity_of_hand_in_odom.twist.angular = ros_helpers.Vec3ToMsg(hand_tform_odom.rotation.transform_vec3(manipulator_state.velocity_of_hand_in_odom.angular))
    manipulator_state_msg.velocity_of_hand_in_odom.header.frame_id = HAND_FRAME_NAME
    manipulator_state_msg.velocity_of_hand_in_odom.header.stamp = ros_stamp.to_msg()
    
    return manipulator_state_msg

def img_msg_to_proto(image_msg: Image, camera_info_msg: CameraInfo, tf_msg: TFMessage, driver: SpotManipulationDriver) -> ImageResponseProto:
    """Takes a ROS Image, CameraInfo, and TF tree representing important transforms for a camera, and populates the corresponding image proto message 

    Assumptions:
        image format is RAW and pixel format is RGB_U8
    Args:
        image_msg: ROS image message
        camera_info: camera_info 
        tf_msg: transforms representing the frame tree snapshot for the associated camera
    Returns:
        image_pb2.ImageResponse: Image proto message
    """

    # Basic image info
    data = image_pb2.ImageResponse() 
    data.shot.acquisition_time = driver._lease_manager.robot.time_sync.robot_timestamp_from_local_secs(image_msg.header.stamp.seconds)
    data.shot.frame_name_image_sensor = image_msg.header.frame_id
    data.shot.image.rows = image_msg.height
    data.shot.image.cols = image_msg.width
    # Assuming that the encoding is rgb_u8
    data.shot.image.format == image_pb2.Image.FORMAT_RAW
    data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8

    # Image data
    data.shot.image.data = image_msg.data

    # Camera info
    data.source.pinhole.intrinsics.focal_length.x = camera_info_msg.k[0] 
    data.source.pinhole.intrinsics.principal_point.x = camera_info_msg.k[2] 
    data.source.pinhole.intrinsics.focal_length.y = camera_info_msg.k[4] 
    data.source.pinhole.intrinsics.principal_point.y = camera_info_msg.k[5] 

    #  Transform tree info
    for tf in tf_msg.transforms:
        transform = data.shot.transforms_snapshot.child_to_parent_edge_map[tf.child_frame_id]
        transform.parent_frame_name = tf.header.frame_id
        transform.parent_tform_child.position.x = tf.transform.translation.x
        transform.parent_tform_child.position.y = tf.transform.translation.y
        transform.parent_tform_child.position.z = tf.transform.translation.z
        transform.parent_tform_child.rotation.x = tf.transform.rotation.x
        transform.parent_tform_child.rotation.y = tf.transform.rotation.y
        transform.parent_tform_child.rotation.z = tf.transform.rotation.z
        transform.parent_tform_child.rotation.w = tf.transform.rotation.w

    return data

def cartesian_request_to_command(msg: ArmCartesianCommand.Goal, tf_buffer: Buffer) -> ArmCartesianCommandProto:

    odom_tform_task = tf_buffer.lookup_transform(ODOM_FRAME_NAME, msg.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp), rclpy.time.Duration(seconds=1.0))
    ref_time = timestamp_pb2.Timestamp().GetCurrentTime()

    # Form the position trajectory
    pose_trajectory = trajectory_pb2.SE3Trajectory(
        points=[
            trajectory_pb2.SE3TrajectoryPoint(
                pose = ros_helpers.MsgToPose(ros_pose).to_proto(),
                time_since_reference = seconds_to_duration(time_offset)
            )
            for (ros_pose, time_offset) in zip(msg.waypoints, msg.timestamps)
        ],
        reference_time = ref_time,
        pos_interpolation = trajectory_pb2.POS_INTERP_CUBIC,
        ang_interpolation = trajectory_pb2.ANG_INTERP_CUBIC_EULER
    )

    # Form the wrench trajectory
    wrench_trajectory = trajectory_pb2.WrenchTrajectory(
        points = [
            trajectory_pb2.WrenchTrajectoryPoint(
                wrench = MsgToWrench(ros_wrench),
                time_since_reference = seconds_to_duration(time_offset)
            )
            for (ros_wrench, time_offset) in zip(msg.wrench_trajectory, msg.timestamps)
        ],
        reference_time = ref_time,
    )

    arm_cartesian_request = arm_command_pb2.ArmCartesianCommand.Request()
    arm_cartesian_request.root_frame_name = ODOM_FRAME_NAME
    arm_cartesian_request.root_tform_task.CopyFrom(ros_helpers.MsgToTransform(odom_tform_task.transform).to_proto())
    arm_cartesian_request.pose_trajectory_in_task.CopyFrom(pose_trajectory)
    if msg.max_acceleration != 0:
        arm_cartesian_request.maximum_acceleration.value = msg.max_acceleration
    if msg.max_linear_velocity != 0:
        arm_cartesian_request.max_linear_velocity.value  = msg.max_linear_velocity
    if msg.max_angular_velocity != 0:
        arm_cartesian_request.max_angular_velocity.value = msg.max_angular_velocity
    if msg.max_pos_tracking_error != 0:
        arm_cartesian_request.max_pos_tracking_error.value = msg.max_pos_tracking_error
    if msg.max_rot_tracking_error != 0:
        arm_cartesian_request.max_rot_tracking_error.value = msg.max_rot_tracking_error
    arm_cartesian_request.force_remain_near_current_joint_configuration = msg.force_remain_near_current_joint_configuration
    arm_cartesian_request.x_axis = msg.x_axis_mode
    arm_cartesian_request.y_axis = msg.y_axis_mode
    arm_cartesian_request.z_axis = msg.z_axis_mode
    arm_cartesian_request.rx_axis = msg.rx_axis_mode
    arm_cartesian_request.ry_axis = msg.ry_axis_mode
    arm_cartesian_request.rz_axis = msg.rz_axis_mode
    arm_cartesian_request.wrench_trajectory_in_task.CopyFrom(wrench_trajectory)

    return arm_cartesian_request
