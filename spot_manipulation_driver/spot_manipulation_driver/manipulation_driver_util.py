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

import argparse
import time
from typing import Text, Tuple

import bosdyn.client
import bosdyn.client.util
import numpy as np
import yaml
from google.protobuf import duration_pb2
from bosdyn.api import (estop_pb2, image_pb2, geometry_pb2, robot_command_pb2,
                        synchronized_command_pb2, arm_command_pb2, robot_state_pb2)
from bosdyn.client import ResponseError, RpcError
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.lease import (InvalidResourceError, LeaseKeepAlive,
                                 NotAuthoritativeServiceError,
                                 ResourceAlreadyClaimedError)
from bosdyn.client.robot_command import (RobotCommandBuilder,
                                         RobotCommandClient,
                                         block_until_arm_arrives,
                                         blocking_stand)
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.util import seconds_to_timestamp
from google.protobuf.timestamp_pb2 import Timestamp

from spot_driver.spot_lease_manager import SpotLeaseManager
from spot_driver.async_queries import AsyncImageService, AsyncRobotState

class SpotManipulationDriver(object):

    # Constructor
    def __init__(self, logger, hostname):
        # Robot Object
        self.logger = logger
        self._is_connected = False

        # Clients
        self._robot_state_client = None
        self._lease_manager = None
        self._image_client = None

        # Tasks
        self._hand_image_task = None
        self._robot_state_task = None

    def connect(self, lease_manager: SpotLeaseManager, rates = {}, callbacks = {}) -> bool:
        if lease_manager is None:
            self.logger.fatal("Cannot connect to robot without a valid lease manager object")

        # Have the base wrapper connect to the robot
        self._lease_manager = lease_manager
        if not self._lease_manager.is_connected:
            self._lease_manager.setLogger(self._logger)
            if not self._lease_manager.connect(self._hostname):
                return False
        
        # Verify that the robot has an arm
        if not self._lease_manager.robot.has_arm():
            self.logger.fatal("Robot requires arm to use SpotManipulationDriver")
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
            self.logger.error(f'Unable to create client service: {e}')
            return False
        
        # Create asynchronous tasks whose state can be queried
        self._hand_image_task = AsyncImageService(self._image_client, self.logger, rates.get("sensors.hand_image", 1.0), callbacks.get("hand_image", lambda:None), hand_image_requests)
        self._robot_state_task  = AsyncRobotState(self._lease_manager._robot_state_client, self.logger, rates.get("status.arm_state", 1.0), callbacks.get("arm_state", lambda:None))
        self._is_connected = True
        return True

    # Authenticate robot
    # def authenticate_robot(self, argv):
    #     # parse arguments
    #     parser = argparse.ArgumentParser()
    #     bosdyn.client.util.add_base_arguments(parser)
    #     config = parser.parse_args(argv)

    #     # Setup logging
    #     bosdyn.client.util.setup_logging(config.verbose)
    #     # create robot and authenticate
    #     sdk = bosdyn.client.create_standard_sdk("SpotManipulationDriverClient")
    #     self._lease_manager.robot = sdk.create_robot(config.hostname)
    #     bosdyn.client.util.authenticate(self._lease_manager.robot)

    # Initialize clients
    # def init_clients(self):
        # self.estop_client = self._lease_manager.robot.ensure_client(EstopClient.default_service_name)
        # self.command_client = self._lease_manager.robot.ensure_client(
        #     RobotCommandClient.default_service_name
        # )
        # self.lease_client = self._lease_manager.robot.ensure_client(
        #     bosdyn.client.lease.LeaseClient.default_service_name
        # )
        # self._robot_state_client = self._lease_manager.robot.ensure_client(
        #     RobotStateClient.default_service_name
        # )

    # Useful getters
    # def get_robot(self):
    #     return self._lease_manager.robot

    @property
    def arm_state(self) -> robot_state_pb2.ManipulatorState:
        return self._robot_state_task.proto.manipulator_state
        # return self._lease_manager._robot_state_client.get_robot_state()

    # def get_joint_states(self):

    #     joint_states_name = []
    #     joint_states_position = []
    #     joint_states_velocity = []
    #     joint_states_effort = []

    #     # Get robot time as local time
    #     state = self.get_robot_state()
    #     local_time = self.get_robot_time_as_local_time(
    #         state.kinematic_state.acquisition_timestamp
    #     )
    #     # Pack joint states into returnable variables
    #     for joint in state.kinematic_state.joint_states:
    #         if joint.name == "arm0.hr0":  # Ignore this joint
    #             continue
    #         joint_states_name.append(
    #             self.joint_names_remapping_dict().get(joint.name, "ERROR")
    #         )
    #         joint_states_position.append(joint.position.value)
    #         joint_states_velocity.append(joint.velocity.value)
    #         joint_states_effort.append(joint.load.value)
    #     return (
    #         local_time,
    #         joint_states_name,
    #         joint_states_position,
    #         joint_states_velocity,
    #         joint_states_effort,
    #     )

    def get_force_torque_state(self):
        state = self.arm_state
        ee_force = state.manipulator_state.estimated_end_effector_force_in_hand
        return ee_force.x, ee_force.y, ee_force.z

    # Remap sdk joint names to easy-to-read names
    # def joint_names_remapping_dict(self):
    #     with open(
    #         # "./../../config/joint_names.yaml", "r"
    #         "/home/spot/manipulation_ws/src/nrg_spot_manipulation/spot_manipulation_driver/config/joint_names.yaml",
    #         "r",
    #     ) as old_joint_names:  # TODO: Use relative path
    #         try:
    #             new_joint_names = yaml.safe_load(old_joint_names)
    #             return new_joint_names
    #         except yaml.YAMLError as exc:
    #             print(exc)

    # Translate robot time to local time: Function borrowed from heuristicus/spot-ros
    # def get_robot_time_as_local_time(self, timestamp):
    #     self._lease_manager.robot.time_sync.wait_for_sync()
    #     rtime = Timestamp()
    #     time_skew = self._lease_manager.robot.time_sync.endpoint.clock_skew
    #     rtime.seconds = timestamp.seconds - time_skew.seconds
    #     rtime.nanos = timestamp.nanos - time_skew.nanos
    #     if rtime.nanos < 0:
    #         rtime.nanos = rtime.nanos + 1000000000
    #         rtime.seconds = rtime.seconds - 1

    #     # Workaround for timestamps being incomplete
    #     if rtime.seconds < 0:
    #         rtime.seconds = 0
    #         rtime.nanos = 0

    #     return rtime

    # Convert trajectory message from ROS MoveIt or other platforms to needed style
    # def convert_ros_trajectory_msg(self, msg):
    #     traj_point_positions = []
    #     traj_point_velocities = []
    #     time_since_ref = []

    #     # Order of joints
    #     joint_order = [
    #         "arm0_shoulder_yaw",
    #         "arm0_shoulder_pitch",
    #         "arm0_elbow_pitch",
    #         "arm0_elbow_roll",
    #         "arm0_wrist_pitch",
    #         "arm0_wrist_roll",
    #     ]

    #     # Reorder joint commands based on joint_order and put them into long lists of lists
    #     for i in range(0, len(msg.trajectory.points)):
    #         pos_dict = {}
    #         vel_dict = {}

    #         for j in range(0, 6):
    #             pos_dict[msg.trajectory.joint_names[j]] = msg.trajectory.points[
    #                 i
    #             ].positions[j]
    #             vel_dict[msg.trajectory.joint_names[j]] = msg.trajectory.points[
    #                 i
    #             ].velocities[j]

    #         traj_point_positions.append(
    #             list(map(lambda joint_name: pos_dict[joint_name], joint_order))
    #         )
    #         traj_point_velocities.append(
    #             list(map(lambda joint_name: vel_dict[joint_name], joint_order))
    #         )
    #         time_since_ref.append(
    #             msg.trajectory.points[i].time_from_start.sec
    #             + msg.trajectory.points[i].time_from_start.nanosec * 1e-9
    #         )

    #     return traj_point_positions, traj_point_velocities, time_since_ref

    # Verify that an e-stop exists: function borrowed from arm_joint_long_trajectory example
    def verify_estop(self):

        status = self._lease_manager.eStopStatus()
        if status.stop_level != estop_pb2.ESTOP_LEVEL_NONE:
            error_message = (
               f"Robot is estopped with message {status.stop_level_details}. Please use an 
                 external E-Stop client, such as the estop SDK example, to configure E-Stop."
            )
            self._lease_manager.robot.logger.error(error_message)
            raise Exception(error_message)

    # Verify that an e-stop exists: function borrowed from arm_joint_long_trajectory example
    def verify_power_and_estop(self):
        if not self._lease_manager.robot.is_powered_on():
            self._lease_manager.robot.logger.info("Robot is not powered on. Attempting to power on.")
            self._lease_manager.robot.power_on(timeout_sec=20)
            assert self._lease_manager.robot.is_powered_on(), "Robot power on failed."
            self._lease_manager.robot.logger.info("Robot powered on.")
        else:
            self._lease_manager.robot.logger.info("Verified that robot is powered on.")

        self.verify_estop()
        # if self.estop_client.get_status().stop_level != estop_pb2.ESTOP_LEVEL_NONE:
        #     error_message = (
        #         "Robot is estopped or estop is not set up. Please configure an e-stop."
        #     )
        #     self._lease_manager.robot.logger.error(error_message)
        #     raise Exception(error_message)

    # Execute long trajectories
    def arm_long_trajectory_executor(
        self, traj_point_positions, traj_point_velocities, time_since_ref
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
        end_index = len(traj_point_positions) - 1

        # Compute reference time for the entire long trajectory
        start_time = time.time() + TRAJ_APPROACH_TIME
        ref_time = seconds_to_timestamp(start_time)

        while traj_index[0] < end_index:
            times = []
            positions = []
            velocities = []

            # Extract a short trajectory from the long list
            times = time_since_ref[traj_index[0] : traj_index[1]]
            positions = traj_point_positions[traj_index[0] : traj_index[1]]
            velocities = traj_point_velocities[traj_index[0] : traj_index[1]]

            # Increment indices for the next short trajectory
            traj_index = list(map(lambda x: x + 9, traj_index))

            if traj_index[1] > end_index:
                traj_index[1] = end_index

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

            cmd_id = self.command_client.robot_command(robot_cmd)

            time_index = time.time()
            feedback_resp = self.command_client.robot_command_feedback(cmd_id)
            joint_move_feedback = (
                feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_joint_move_feedback
            )
            time_to_goal: duration_pb2.Duration = joint_move_feedback.time_to_goal
            time_to_goal_in_seconds: float = time_to_goal.seconds + (
                float(time_to_goal.nanos) / float(10 ** 9)
            )

    def gripper_trajectory_executor(self, traj_point_positions, time_since_ref):

        self._lease_manager.robot.time_sync.wait_for_sync()

        self.verify_power_and_estop()

        self.logger.info("Gripper is about to move.")

        # Retrieve endpoint of the trajectory and execute it
        end_index = len(traj_point_positions) - 1
        position = traj_point_positions[end_index]
        robot_cmd = RobotCommandBuilder.claw_gripper_open_angle_command(position)
        self.command_client.robot_command(robot_cmd)
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

        self.command_client.robot_command(robot_cmd)
        time.sleep(1.0)

        traj_index = 1
        end_index = len(traj_point_positions) - 1

        # Execute gripper trajectory accounting for time differences between points
        while traj_index <= end_index:

            # Extract a point at a time and execute
            positions = traj_point_positions[traj_index]

            robot_cmd = RobotCommandBuilder.claw_gripper_open_angle_command(positions)
            self.command_client.robot_command(robot_cmd)

            time.sleep(time_since_ref[traj_index] - time_since_ref[traj_index - 1])
            traj_index = traj_index + 1

    def ee_velocity_msg_executor(self, msg):
        # Constraints and scale
        scale = 0.5
        drift = 0.1
        linear_vel_min = -0.2
        linear_vel_max = 0.2
        angular_vel_min = -0.5
        angular_vel_max = 0.5
        linear_vel = np.clip(
            np.array([msg.linear.x, msg.linear.y, msg.linear.z]) * scale,
            linear_vel_min,
            linear_vel_max,
        )
        angular_vel = np.clip(
            np.array([msg.angular.x, msg.angular.y, msg.angular.z]) * scale,
            angular_vel_min,
            angular_vel_max,
        )

        # Construct message and send it to the robot
        linear = geometry_pb2.Vec3(x=linear_vel[0], y=linear_vel[1], z=linear_vel[2])
        angular = geometry_pb2.Vec3(
            x=angular_vel[0], y=angular_vel[1], z=angular_vel[2]
        )

        end_time = seconds_to_timestamp(time.time() + drift)

        end_effector_velocity = arm_command_pb2.ArmVelocityCommand.CartesianVelocity(
            frame_name="body", velocity_in_frame_name=linear
        )
        arm_velocity_command = arm_command_pb2.ArmVelocityCommand.Request(
            cartesian_velocity=end_effector_velocity,
            angular_velocity_of_hand_rt_odom_in_hand=angular,
            end_time=end_time,
        )
        arm_command = arm_command_pb2.ArmCommand.Request(
            arm_velocity_command=arm_velocity_command
        )
        synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
            arm_command=arm_command
        )
        robot_cmd = robot_command_pb2.RobotCommand(
            synchronized_command=synchronized_command
        )

        self.command_client.robot_command(robot_cmd)

    # def stand_robot(self):
    #     self._lease_manager.robot.logger.info("Commanding robot to stand...")
    #     blocking_stand(self.command_client, timeout_sec=10)
    #     self._lease_manager.robot.logger.info("Robot standing.")

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
        robot_cmd = RobotCommandBuilder.claw_gripper_close_command()
        (success, msg, id) = self._lease_manager.robot_command(robot_cmd)
        return success, msg
    
    def close_gripper(self) -> Tuple[bool, Text]:
        robot_cmd = RobotCommandBuilder.claw_gripper_open_command()
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

    # def safe_power_off(self):
    #     self._lease_manager.robot.power_off(cut_immediately=False, timeout_sec=20)
    #     assert not self._lease_manager.robot.is_powered_on(), "Robot power off failed."
    #     self._lease_manager.robot.logger.info("Robot safely powered off.")

    # def forceGetLease(self):
    #     try:
    #         self.lease = self.lease_client.acquire()
    #     except (InvalidResourceError, NotAuthoritativeServiceError) as err:
    #         return False, err.error_message

    #     self.lease_keepalive = LeaseKeepAlive(self.lease_client, must_acquire=True)

    # def forceClaim(self) -> Tuple[bool, Text]:
    #     """Get a lease for the robot, a handle on the estop endpoint, and the ID of the robot."""
    #     try:
    #         self.resetEStop()
    #         if not self.forceGetLease():
    #             return False
    #         self.resetEStop()
    #     except (ResponseError, RpcError) as err:
    #         return False, err.error_message

    #     return True, "Success"

    # Functions that follow have been borrowed from NRG's ROS2 Spot wrapper. TODO: Inherit them from the SpotWrapper class

    # def getLease(self) -> Tuple[bool, Text]:
    #     """Get a lease for the robot and keep the lease alive automatically."""
    #     try:
    #         self.lease = self.lease_client.acquire()
    #     except (
    #         ResourceAlreadyClaimedError,
    #         InvalidResourceError,
    #         NotAuthoritativeServiceError,
    #     ) as err:
    #         return False, err.error_message

    #     self.lease_keepalive = LeaseKeepAlive(self.lease_client, must_acquire=True)

    #     return True, "Success"

    def claim(self) -> Tuple[bool, Text]:
        """Get a lease for the robot, a handle on the estop endpoint, and the ID of the robot."""
        if self._lease_manager is None:
            msg = "Cannot claim a lease without first connecting to a LeaseManager"
            return False, msg
        
        result = self._lease_manager.registerLeaseOwner(id(self))
        return result
        # try:
        #     if not self.getLease():
        #         return False
        #     self.resetEStop()
        # except (ResponseError, RpcError) as err:
        #     return False, err.error_message

        # return True, "Success"

    # def resetEStop(self) -> None:
    #     """Get keepalive for eStop"""
    #     self.estop_endpoint = EstopEndpoint(self.estop_client, "ros", 9.0)
    #     self.estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.
    #     self.estop_keepalive = EstopKeepAlive(self.estop_endpoint)

    # def releaseLease(self) -> None:
    #     """Return the lease on the body."""
    #     if self.lease:
    #         self.lease_client.return_lease(self.lease)
    #         self.lease = None

    def release(self) -> bool:
        """Return the lease on the body and the eStop handle."""
        try:
            self._lease_manager.disconnect(id(self))
            # self.releaseLease()
            # self.releaseEStop()
        except Exception as err:
            return False, Text(err)

        return True, "Success"

    # def disconnect(self) -> None:
    #     """Release control of robot as gracefully as posssible."""
    #     if self._lease_manager.robot is None:
    #         return

    #     if self._lease_manager.robot.time_sync:
    #         self._lease_manager.robot.time_sync.stop()
    #     self.stow_arm()
    #     self.safe_power_off()
    #     self.release()

    # def releaseEStop(self) -> None:
    #     """Stop eStop keepalive"""
    #     if self.estop_keepalive:
    #         self.estop_keepalive.stop()
    #         self.estop_keepalive = None
    #         self.estop_endpoint = None
