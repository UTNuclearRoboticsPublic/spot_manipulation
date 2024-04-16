#! /usr/bin/env python
##############################################################################
#      Title     : manipulation_driver_util.py
#      Project   : spot_manipulation_driver
#      Created   : 01/15/2023
#      Author    : Janak Panthi (Crasun Jans)
#      Copyright : Copyright© The University of Texas at Austin, 2023-2030. All
#      rights reserved.
#
#          All files within this directory are subject to the following, unless
#          an alternative license is explicitly included within the text of
#          each file.
#
#          This software and documentation constitute an unpublished work
#          and contain valuable trade secrets and proprietary information
#          belonging to the University. None of the foregoing material may be
#          copied or duplicated or disclosed without the express, written
#          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
#          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
#          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
#          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
#          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
#          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
#          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
#          University be liable for incidental, special, indirect, direct or
#          consequential damages or loss of profits, interruption of business,
#          or related expenses which may arise from use of software or
#          documentation, including but not limited to those resulting from
#          defects in software and/or documentation, or loss or inaccuracy of
#          data of any kind.
##############################################################################

import time
from typing import Text, Tuple
import numpy as np
import geometry_msgs.msg
import scipy
import copy

from google.protobuf import duration_pb2, timestamp_pb2
from bosdyn.api import (estop_pb2,geometry_pb2, image_pb2, robot_command_pb2,
                        synchronized_command_pb2, arm_command_pb2, robot_state_pb2,
                        # arm_surface_contact_pb2, arm_surface_contact_service_pb2,
                        trajectory_pb2, point_cloud_pb2, image_pb2)
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
# from bosdyn.client.arm_surface_contact import ArmSurfaceContactClient
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME, 
                                         GROUND_PLANE_FRAME_NAME, ODOM_FRAME_NAME,
                                         HAND_FRAME_NAME, get_a_tform_b)
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.point_cloud import PointCloudClient
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         blocking_stand, TimedOutError, CommandFailedErrorWithFeedback)
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client import image
from bosdyn.util import seconds_to_timestamp, seconds_to_duration

from spot_driver.spot_lease_manager import SpotLeaseManager
from spot_driver.async_queries import AsyncImageService, AsyncRobotState

class SpotManipulationDriver(object):

    # Constructor
    def __init__(self, logger, hostname):
        # Configuration
        self._logger = logger
        self._is_connected = False
        self._hostname = hostname

        # Clients
        self._robot_state_client = None
        self._lease_manager = None
        self._image_client = None

        # Tasks
        self._hand_image_task = None
        self._robot_state_task = None

    def connect(self, lease_manager: SpotLeaseManager, rates = {}, callbacks = {}) -> bool:
        if lease_manager is None:
            self._logger.fatal("Cannot connect to robot without a valid lease manager object")

        # Have the base wrapper connect to the robot
        self._lease_manager = lease_manager
        if not self._lease_manager.is_connected:
            self._lease_manager.setLogger(self._logger)
            if not self._lease_manager.connect(self._hostname, rates, callbacks):
                return False

        # Verify that the robot has an arm
        if not self._lease_manager.robot.has_arm():
            self._logger.fatal("Robot requires arm to use SpotManipulationDriver")
            return False

        # Configure the hand to publish its depth and color images
        hand_image_sources = {'hand_image', 'hand_depth', 'hand_color_image', 'hand_depth_in_hand_color_frame'}
        hand_image_requests = []
        for source in hand_image_sources:
            hand_image_requests.append(build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW))

        # Start the service clients
        try:
            self._image_client = self._lease_manager.robot.ensure_client(ImageClient.default_service_name)
        except Exception as e:
            self._logger.error(f'Unable to create client service: {e}')
            return False

        # Create asynchronous tasks whose state can be queried
        self._hand_image_task = AsyncImageService(self._image_client, self._logger, rates.get("hand_image", 1.0), callbacks.get("hand_image", lambda:None), hand_image_requests)
        self._robot_state_task = AsyncRobotState(self._lease_manager._robot_state_client, self._logger, rates.get("robot_state", 1.0), callbacks.get("robot_state", lambda:None))
        self._is_connected = True
        return True

    @property
    def robot_state(self) -> robot_state_pb2:
        return self._robot_state_task.proto

    @property
    def kinematic_state(self) -> robot_state_pb2.KinematicState:
        if self._robot_state_task.proto is None: return None
        return self._robot_state_task.proto.kinematic_state

    @property
    def arm_state(self) -> robot_state_pb2.ManipulatorState:
        if self._robot_state_task.proto is None: return None
        return self._robot_state_task.proto.manipulator_state

    @property
    def hand_force(self):
        state = self.arm_state
        if state is None: return None
        ee_force = state.manipulator_state.estimated_end_effector_force_in_hand
        return ee_force.x, ee_force.y, ee_force.z

# TODO
    # @property
    # def arm_impedance(self) -> arm_command_pb2.ArmImpedanceCommand:
    #     if self._robot_state_task.proto is None: return None
    #     return self.arm_impedance
    
    @property
    def robot_time(self) -> timestamp_pb2.Timestamp:
        return self._lease_manager.robot.time_sync.robot_timestamp_from_local_secs(time.time())

    @property
    def latest_hand_images(self):
        return self._hand_image_task.proto

    @property
    def lease_manager(self):
        return self._lease_manager

    # Verify that an e-stop exists: function borrowed from arm_joint_long_trajectory example
    def verify_estop(self):

        status = self._lease_manager.eStopStatus()
        if status.stop_level != estop_pb2.ESTOP_LEVEL_NONE:
            error_message = (
               f"Robot is estopped with message {status.stop_level_details}. Please use an "
                "external E-Stop client, such as the estop SDK example, to configure E-Stop."
            )
            self._lease_manager.robot.logger.error(error_message)
            raise Exception(error_message)

    # Verify that an e-stop exists: function borrowed from arm_joint_long_trajectory example
    def verify_power_and_estop(self):
        if not self._lease_manager.robot.is_powered_on():
            self._lease_manager.logger.info("Robot is not powered on. Attempting to power on.")
            self._lease_manager.robot.power_on(timeout_sec=20)
            assert self._lease_manager.robot.is_powered_on(), "Robot power on failed."
            self._lease_manager.logger.info("Robot powered on.")
        else:
            self._lease_manager.robot.logger.info("Verified that robot is powered on.")

        self.verify_estop()

    # Execute long trajectories
    def arm_long_trajectory_executor(
        self, traj_point_positions, traj_point_velocities, timepoints
    ):

        self._lease_manager.robot.time_sync.wait_for_sync()
        self.verify_power_and_estop()
        self._lease_manager.robot.logger.info("Arm is about to move.")

        # Get to the start configuration of the trajectory before we execute it
        start_time = time.time()
        ref_time = seconds_to_timestamp(start_time)
        TRAJ_APPROACH_TIME = 1.0

        robot_cmd = RobotCommandBuilder.arm_joint_move_helper(
            joint_positions=[traj_point_positions[0]],
            joint_velocities=[traj_point_velocities[0]],
            times=[TRAJ_APPROACH_TIME],
            ref_time=ref_time,
            max_acc=10000,
            max_vel=10000,
        )

        self._lease_manager.robot_command(robot_cmd)

        traj_index = [0, 9]
        end_index = len(traj_point_positions)

        # Compute reference time for the entire long trajectory
        start_time = time.time() + TRAJ_APPROACH_TIME
        ref_time = seconds_to_timestamp(start_time)

        while traj_index[0] < end_index:
            times = []
            positions = []
            velocities = []

            # Don't let the extracted index range go beyond the end of the trajectory
            if traj_index[1] > end_index:
                traj_index[1] = end_index

            # Extract a short trajectory from the long list
            times = timepoints[traj_index[0] : traj_index[1]]
            positions = traj_point_positions[traj_index[0] : traj_index[1]]
            velocities = traj_point_velocities[traj_index[0] : traj_index[1]]

            # Increment indices for the next short trajectory
            traj_index = list(map(lambda x: x + 9, traj_index))


            robot_cmd = RobotCommandBuilder.arm_joint_move_helper(
                joint_positions=positions,
                joint_velocities=velocities,
                times=times,
                ref_time=ref_time,
                max_acc=10000,
                max_vel=10000,
            )

            # Compute sleep time and sleep before executing next trajectory
            if traj_index[0] > 9:
                time.sleep(time_to_goal_in_seconds - (time.time() - time_index) - 0.05)

            success, msg, cmd_id = self._lease_manager.robot_command(robot_cmd)

            time_index = time.time()
            feedback_resp = self._lease_manager.robot_command_feedback(cmd_id)
            joint_move_feedback = (
                feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_joint_move_feedback
            )
            time_to_goal: duration_pb2.Duration = joint_move_feedback.time_to_goal
            time_to_goal_in_seconds: float = time_to_goal.seconds + (
                float(time_to_goal.nanos) / float(10 ** 9)
            )

    def arm_force_trajectory_executor(
            self, distance_x, distance_y, distance_z, 
            force_x, force_y, force_z, speed):
        self._lease_manager.robot.time_sync.wait_for_sync()
        command_client = self._lease_manager.command_client

        hand_pose_in_body_frame: geometry_pb2.SE3Pose = get_a_tform_b(self.kinematic_state.transforms_snapshot,
                              GRAV_ALIGNED_BODY_FRAME_NAME, HAND_FRAME_NAME)
        
        hand_x_start = hand_pose_in_body_frame.position.x  # in front of the robot.
        hand_y_start = hand_pose_in_body_frame.position.y  # to the left
        hand_z_start = hand_pose_in_body_frame.position.z  # will be ignored since we'll have a force in the Z axis.
        hand_x_end = hand_x_start + distance_x
        hand_y_end = hand_y_start + distance_y  # to the right
        hand_z_end = hand_z_start + distance_z

        f_x = force_x
        f_y = force_y
        f_z = force_z  # Newtons

        hand_vec3_start = geometry_pb2.Vec3(x=hand_x_start, y=hand_y_start, z=hand_z_start)
        hand_vec3_end = geometry_pb2.Vec3(x=hand_x_end, y=hand_y_end, z=hand_z_end)

        q = Quat.from_pitch(1.571) # for pointing eef downward
        # q = Quat.from_pitch(0) # for eef pointing forward
        quat = q.to_proto()
        # qw = 1
        # qx = 0
        # qy = 0
        # qz = 0
        # quat = geometry_pb2.Quaternion(w=qw, x=qx, y=qy, z=qz)

        # Build a position trajectory
        hand_pose1_in_flat_body = geometry_pb2.SE3Pose(position=hand_vec3_start, rotation=quat)
        hand_pose2_in_flat_body = geometry_pb2.SE3Pose(position=hand_vec3_end, rotation=quat)

        distance = np.linalg.norm([distance_x,distance_y,distance_z])

        traj_time = distance/speed
        self._logger.info(f"trajectory time: {traj_time}")
        time_since_reference = seconds_to_duration(traj_time)

        # Set arm against ground
        command = RobotCommandBuilder.arm_wrench_command(f_x, f_y, f_z, 
                                                         0, 0, 0,
                                                         ODOM_FRAME_NAME, traj_time)
        # Send the request
        command_client.robot_command(command)

        # Convert the poses to the odometry frame.
        odom_T_flat_body = get_a_tform_b(self.kinematic_state.transforms_snapshot,
                                         ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
        hand_pose1 = odom_T_flat_body * SE3Pose.from_proto(hand_pose1_in_flat_body)
        hand_pose2 = odom_T_flat_body * SE3Pose.from_proto(hand_pose2_in_flat_body)


        traj_point1 = trajectory_pb2.SE3TrajectoryPoint(pose=hand_pose1.to_proto(),
                                                        time_since_reference=seconds_to_duration(0))
        traj_point2 = trajectory_pb2.SE3TrajectoryPoint(pose=hand_pose2.to_proto(),
                                                        time_since_reference=time_since_reference)

        hand_traj = trajectory_pb2.SE3Trajectory(points=[traj_point1, traj_point2])

        # We'll use a constant wrench here.  Build a wrench trajectory with a single point.
        # Note that we only need to fill out Z-force because that is the only axis that will
        # be in force mode.
        force_tuple = odom_T_flat_body.rotation.transform_point(x=f_x, y=f_y, z=f_z)
        force = geometry_pb2.Vec3(x=force_tuple[0], y=force_tuple[1], z=force_tuple[2])
        torque = geometry_pb2.Vec3(x=0, y=0, z=0)

        wrench = geometry_pb2.Wrench(force=force, torque=torque)

        # We set this point to happen at 0 seconds.  The robot will hold the wrench past that
        # time, so we'll end up with a constant value.
        duration = seconds_to_duration(0)

        traj_point = trajectory_pb2.WrenchTrajectoryPoint(wrench=wrench,
                                                          time_since_reference=duration)
        trajectory = trajectory_pb2.WrenchTrajectory(points=[traj_point])

        arm_cartesian_command = arm_command_pb2.ArmCartesianCommand.Request(
            pose_trajectory_in_task=hand_traj, root_frame_name=ODOM_FRAME_NAME,
            wrench_trajectory_in_task=trajectory,
            x_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_POSITION,
            y_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_POSITION,
            z_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
            rx_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_POSITION,
            ry_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_POSITION,
            rz_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_POSITION)
        self._logger.info(f'cartesian command: {arm_cartesian_command}')
        arm_command = arm_command_pb2.ArmCommand.Request(
            arm_cartesian_command=arm_cartesian_command)
        synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
            arm_command=arm_command)
        robot_command = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)

        success,msg = command_client.robot_command(robot_command)
        self._lease_manager.logger.info('Running mixed position and force mode.')
        
        hand_pose_in_body_frame_end: geometry_pb2.SE3Pose = get_a_tform_b(self.kinematic_state.transforms_snapshot,
                              GRAV_ALIGNED_BODY_FRAME_NAME, HAND_FRAME_NAME)
        error = hand_vec3_end - hand_pose_in_body_frame_end.position

    def force_transform(self, wrench):
        
        force_vector = geometry_pb2.Vec3(x=wrench.force.x, y=wrench.force.y, z=wrench.force.z)
        torque_vector = geometry_pb2.Vec3(x=wrench.torque.x, y=wrench.torque.y, z=wrench.torque.z)      

        sensor_in_odom: SE3Pose = get_a_tform_b(self.kinematic_state.transforms_snapshot,
                                GRAV_ALIGNED_BODY_FRAME_NAME, HAND_FRAME_NAME)
        
        force_transformed = sensor_in_odom.transform_vec3(force_vector)
        torque_transformed = sensor_in_odom.transform_vec3(torque_vector)

        force_in_odom = geometry_msgs.msg.Vector3(x=force_transformed.x, y=force_transformed.y, z=force_transformed.z)
        torque_in_odom = geometry_msgs.msg.Vector3(x=torque_transformed.x, y=torque_transformed.y, z=torque_transformed.z)

        wrench_in_odom = geometry_msgs.msg.Wrench(force=force_in_odom, torque=torque_in_odom)

        return wrench_in_odom

    def current_location(self):
        current_location_body_frame: SE3Pose = get_a_tform_b(self.kinematic_state.transforms_snapshot,
                            GRAV_ALIGNED_BODY_FRAME_NAME, HAND_FRAME_NAME)
        return current_location_body_frame
    
    def arm_ncontact_trajectory_executor(
            self, step_distance, offset, speed,
            i, steps, old, new, distance):
        
        # self._logger.info('in ncontact trajectory executor')
        # self._logger.info(f'step distance: {step_distance}'+ '\n' +
        #                   f'offset: {offset}'+ '\n' +
        #                   f'speed: {speed}'+ '\n' +
        #                   f'i: {i}'+ '\n' +
        #                   f'steps: {steps}'+ '\n' +
        #                   f'old: {old}'+ '\n' +
        #                   f'new: {new}'+ '\n' +
        #                   f'distance: {distance}')
        
        old = geometry_pb2.Vec3(x=old.x, y=old.y, z=old.z)
        new = geometry_pb2.Vec3(x=new.x, y=new.y, z=new.z)
        
        self._lease_manager.robot.time_sync.wait_for_sync()
        command_client = self._lease_manager.command_client

        
        # q = Quat.from_pitch(1.571) # for pointing eef downward
        q = Quat.from_pitch(0) # for eef pointing forward
        quat = q.to_proto()

        hand_pose_in_body_frame: SE3Pose = get_a_tform_b(self.kinematic_state.transforms_snapshot,
                            GRAV_ALIGNED_BODY_FRAME_NAME, HAND_FRAME_NAME)

        if i == 0:
            hand_x_start = hand_pose_in_body_frame.x
            hand_y_start = hand_pose_in_body_frame.y
            hand_z_start = hand_pose_in_body_frame.z

            hand_x_end = hand_x_start + step_distance.x*steps
            hand_y_end = hand_y_start + step_distance.y*steps
            hand_z_end = hand_z_start + step_distance.z*steps
            hand_vec3_end = geometry_pb2.Vec3(x=hand_x_end, y=hand_y_end, z=hand_z_end)
            # self._logger.info('start position saved')

        hand_x_current = hand_pose_in_body_frame.x  # in front of the robot.
        hand_y_current = hand_pose_in_body_frame.y  # to the left
        hand_z_current = hand_pose_in_body_frame.z  # will be ignored since we'll have a force in the Z axis.
        hand_vec3_current = geometry_pb2.Vec3(x=hand_x_current, y=hand_y_current, z=hand_z_current)

        perception_distance = geometry_pb2.Vec3(x=(new.x-old.x), y=(new.y-old.y), z=(new.z-old.z))

        hand_x_predicted = new.x - offset.x
        hand_y_predicted = hand_y_current + step_distance.y 
        hand_z_predicted = new.z + offset.z
        # hand_z_predicted = hand_z_current

        self._logger.info(f'new x: {new.x}, offset x: {offset.x}')

        hand_pose_in_body_frame_predicted = copy.copy(hand_pose_in_body_frame)
        hand_vec3_predicted = geometry_pb2.Vec3(x=hand_x_predicted, y=hand_y_predicted, z=hand_z_predicted)
        # self._logger.info(f'predicted position: {hand_vec3_predicted}')
        hand_pose_in_body_frame_predicted.x = hand_vec3_predicted.x
        hand_pose_in_body_frame_predicted.y = hand_vec3_predicted.y
        hand_pose_in_body_frame_predicted.z = hand_vec3_predicted.z

        self._logger.info(f'HTMs calculated'+ '\n' +
                          f'current: {hand_pose_in_body_frame}' + '\n' +
                          f'predicted: {hand_pose_in_body_frame_predicted}')
        # self._logger.info(f'current: {hand_pose_in_body_frame}')
        # self._logger.info(f'predicted: {hand_pose_in_body_frame_predicted}')

        # skew_twist = scipy.linalg.logm(geometry_pb2.SE3Pose.from_proto(hand_pose_in_body_frame_predicted) * 
        #                                SE3Pose.inverse(geometry_pb2.SE3Pose.from_proto(hand_pose_in_body_frame)))
        skew_twist = scipy.linalg.logm((hand_pose_in_body_frame_predicted * hand_pose_in_body_frame.inverse()).to_matrix())
        linear_velocity= np.array([skew_twist[0,3], # Hand frame
                    skew_twist[1,3],
                    skew_twist[2,3]])
        
        self._logger.info(f'skew SE3 calculated: {skew_twist}'+ '\n' +
                          f'velocity calculated: {linear_velocity}')
        linear_velocity[1] = speed
        if step_distance.y < 0:
            linear_velocity[1] = linear_velocity[1]*-1
            
        lim = 0.02
        if abs(linear_velocity[0]) < lim and abs(hand_x_current - hand_x_predicted) > 0.01:
            linear_velocity[0] = linear_velocity[0]*(lim/abs(linear_velocity[2]))
        if abs(linear_velocity[2]) < lim and abs(hand_z_current - hand_z_predicted) > 0.01:
            linear_velocity[2] = linear_velocity[2]*(lim/abs(linear_velocity[2]))

        if step_distance.x == 0: linear_velocity[0] = 0
        # if step_distance.z == 0: linear_velocity[2] = 0
        elif step_distance.y == 0: linear_velocity[1] = 0

        # magnitude = np.linalg.norm(linear_velocity)
        # if  magnitude > speed:
        #     linear_velocity = linear_velocity*(speed/magnitude)

        self._logger.info(f'linear velocity limited: {linear_velocity}')
        linear = geometry_msgs.msg.Vector3(x=linear_velocity[0], y=linear_velocity[1], z=linear_velocity[2])

        linear_velocity = geometry_pb2.Vec3(x=linear_velocity[0], y=linear_velocity[1], z=linear_velocity[2])
        angular_velocity = np.array([skew_twist[2,1], # Hand frame
                    skew_twist[0,2],
                    skew_twist[1,0]])
        self._logger.info(f'angular velocity calculated: {angular_velocity}')
        
        magnitude = np.linalg.norm(angular_velocity)
        if  magnitude > speed:
            angular_velocity = angular_velocity*(speed/magnitude)
        angular = geometry_msgs.msg.Vector3(x=angular_velocity[0], y=angular_velocity[1], z=angular_velocity[2])
        twist_msg=geometry_msgs.msg.Twist(linear=linear)#, angular=angular)
        self._logger.info(f'angular velocity limited: {angular_velocity}')
        angular_velocity = geometry_pb2.Vec3(x=angular_velocity[0], y=angular_velocity[1], z=angular_velocity[2])
        # Get to the start configuration of the trajectory before we execute it

        self._logger.info(f'velocity:{linear_velocity}')
        self._logger.info(f'twist: {twist_msg}')

        # hand_pose_in_body_frame_end: geometry_pb2.SE3Pose =s, time_since_ref):
        #                         BODY_FRAME_NAME, HAND_FRAME_NAME)
        # error = hand_vec3_end - hand_pose_in_body_frame_end.position

        return twist_msg

    def transform_frame(self, point, frames):
        self._logger.info(f"point: {point}, frames: {frames}")
        point_coord = geometry_pb2.Vec3(x=point.x, y=point.y, z=point.z)

        for frame in frames:
            self._logger.info('correcting frame names')
            if frame == 'hand':
                frame = HAND_FRAME_NAME
            elif frame == 'body':
                frame = BODY_FRAME_NAME

        sensor_in_new_frame: SE3Pose = get_a_tform_b(self.kinematic_state.transforms_snapshot,
                               frames[0] ,frames[1])
        self._logger.info("Got a_tform_b")
        
        point_transformed = sensor_in_new_frame.transform_vec3(point_coord)

        point = geometry_msgs.msg.Point(x=point_transformed.x, y=point_transformed.y, z=point_transformed.z)
        self._logger.info("returning point")

        return point

    def gripper_trajectory_executor(self, traj_point_positions, time_since_ref):

        self._lease_manager.robot.time_sync.wait_for_sync()

        self.verify_power_and_estop()

        self._logger.info("Gripper is about to move.")

        # Retrieve endpoint of the trajectory and execute it
        end_index = len(traj_point_positions) - 1
        position = traj_point_positions[end_index]
        robot_cmd = RobotCommandBuilder.claw_gripper_open_angle_command(position)
        self._lease_manager.robot_command(robot_cmd)
        time.sleep(time_since_ref[end_index])

    def gripper_trajectory_executor_with_time_control(
        self, traj_point_positions, time_since_ref
    ):
        self._lease_manager.robot.time_sync.wait_for_sync()

        self.verify_power_and_estop()

        self._lease_manager.robot.logger.info("Gripper is about to move.")

        # Get to the start configuration of the trajectory before we execute it
        robot_cmd = RobotCommandBuilder.claw_gripper_open_angle_command(
            traj_point_positions[0]
        )

        self._lease_manager.robot_command(robot_cmd)
        time.sleep(1.0)

        traj_index = 1
        end_index = len(traj_point_positions) - 1

        # Execute gripper trajectory accounting for time differences between points
        while traj_index <= end_index:

            # Extract a point at a time and execute
            positions = traj_point_positions[traj_index]

            robot_cmd = RobotCommandBuilder.claw_gripper_open_angle_command(positions)
            self._lease_manager.robot_command(robot_cmd)

            time.sleep(time_since_ref[traj_index] - time_since_ref[traj_index - 1])
            traj_index = traj_index + 1

    def ee_velocity_msg_executor(self, request: arm_command_pb2.ArmVelocityCommand.Request) -> Tuple[bool, Text]:

        arm_command = arm_command_pb2.ArmCommand.Request(
            arm_velocity_command=request
        )
        synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
            arm_command=arm_command
        )
        robot_cmd = robot_command_pb2.RobotCommand(
            synchronized_command=synchronized_command
        )
        self._logger.info(f'arm command: {request}')
        (success, msg, id) = self._lease_manager.robot_command(robot_cmd)
        return success, msg

    def stand_robot(self) -> Tuple[bool, Text]:
        self._lease_manager.robot.logger.info("Commanding robot to stand...")
        try:
            blocking_stand(self._lease_manager.command_client, timeout_sec=10)
        except CommandFailedErrorWithFeedback as error:
            return False, error.message
        except TimedOutError as error:
            return False, error.error_message
        return True, 'Robot standing'
    
    def stand_robot_body_assist(self) -> Tuple[bool, Text]:
        self._lease_manager.robot.logger.info("Commanding robot to stand...")
        command_client = self._lease_manager.command_client

        try:
            body_control = spot_command_pb2.BodyControlParams(
                body_assist_for_manipulation=spot_command_pb2.BodyControlParams.
                BodyAssistForManipulation(enable_hip_height_assist=True, enable_body_yaw_assist=True))
            blocking_stand(command_client, timeout_sec=10,
                       params=spot_command_pb2.MobilityParams(body_control=body_control))
        except CommandFailedErrorWithFeedback as error:
            return False, error.message
        except TimedOutError as error:
            return False, error.error_message
        return True, 'Robot standing'

    def stow_arm(self) -> Tuple[bool, Text]:
        robot_cmd = RobotCommandBuilder.arm_stow_command()
        (success, msg, id) = self._lease_manager.robot_command(robot_cmd)
        # block_until_arm_arrives(self._lease_manager.command_client, cmd_id)
        return success, msg

    def unstow_arm(self) -> Tuple[bool, Text]:
        robot_cmd = RobotCommandBuilder.arm_ready_command()
        (success, msg, id) = self._lease_manager.robot_command(robot_cmd)
        return success, msg

    def open_gripper(self) -> Tuple[bool, Text]:
        robot_cmd = RobotCommandBuilder.claw_gripper_open_command()
        (success, msg, id) = self._lease_manager.robot_command(robot_cmd)
        return success, msg

    def close_gripper(self) -> Tuple[bool, Text]:
        robot_cmd = RobotCommandBuilder.claw_gripper_close_command()
        (success, msg, id) = self._lease_manager.robot_command(robot_cmd)
        return success, msg

    def open_gripper_to_angle(self, angle: float) -> Tuple[bool, Text]:
        if angle > 90.0 or angle < 0.0:
            return False, Text('Could not set gripper angle to invalid angle' + angle)

        # The open angle command does not take degrees but the limits
        # defined in the urdf, that is why we have to interpolate
        closed = 0.349066
        opened = -1.396263
        angle = angle / 90.0 * (opened - closed) + closed

        (success, msg, id) = robot_cmd = RobotCommandBuilder.claw_gripper_open_angle_command(angle)
        self._lease_manager.robot_command(robot_cmd)
        return success, msg

    def claim(self) -> Tuple[bool, Text]:
        """Get a lease for the robot, a handle on the estop endpoint, and the ID of the robot."""
        if self._lease_manager is None:
            msg = "Cannot claim a lease without first connecting to a LeaseManager"
            return False, msg

        result = self._lease_manager.registerLeaseOwner(id(self))
        return result

    def release(self) -> bool:
        """Return the lease on the body and the eStop handle."""
        try:
            self.stow_arm()
            self._lease_manager.disconnect(id(self))
        except Exception as err:
            return False, Text(err)

        return True, "Success"
