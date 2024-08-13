import numpy as np
from typing import Tuple, List
from bosdyn.api import arm_command_pb2, geometry_pb2, robot_state_pb2, image_pb2
from bosdyn.client.math_helpers import SE3Pose, Quat
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from google.protobuf import timestamp_pb2
from spot_msgs.msg import ManipulatorState
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rclpy.time

from .spot_manipulation_driver import SpotManipulationDriver

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

def img_msg_to_proto(image_msg: Image, camera_info: CameraInfo, tf_msg: TFMessage, driver: SpotManipulationDriver) -> image_pb2.ImageResponse:
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
    data.shot.acquisition_time = drive._lease_manager.robot.time_sync.robot_timestamp_from_local_secs(image_msg.header.stamp.seconds)
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
