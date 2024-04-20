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

from bosdyn.api import (arm_command_pb2, mobility_command_pb2, estop_pb2, image_pb2,
                        robot_command_pb2, robot_state_pb2,
                        synchronized_command_pb2, geometry_pb2)
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.robot_command import (CommandFailedErrorWithFeedback,
                                         RobotCommandBuilder, TimedOutError,
                                         blocking_stand)
from bosdyn.util import seconds_to_timestamp, seconds_to_duration
from google.protobuf import duration_pb2, timestamp_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2

from spot_driver.async_queries import AsyncImageService, AsyncRobotState
from spot_driver.spot_lease_manager import SpotLeaseManager
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2


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

    def connect(self, lease_manager: SpotLeaseManager, rates={}, callbacks={}) -> bool:
        if lease_manager is None:
            self._logger.fatal(
                "Cannot connect to robot without a valid lease manager object"
            )

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
        hand_image_sources = {
            "hand_image",
            "hand_depth",
            "hand_color_image",
            "hand_depth_in_hand_color_frame",
        }
        hand_image_requests = []
        for source in hand_image_sources:
            hand_image_requests.append(
                build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW)
            )

        # Start the service clients
        try:
            self._image_client = self._lease_manager.robot.ensure_client(
                ImageClient.default_service_name
            )
        except Exception as e:
            self._logger.error(f"Unable to create client service: {e}")
            return False

        # Create asynchronous tasks whose state can be queried
        self._hand_image_task = AsyncImageService(
            self._image_client,
            self._logger,
            rates.get("hand_image", 1.0),
            callbacks.get("hand_image", lambda: None),
            hand_image_requests,
        )
        self._robot_state_task = AsyncRobotState(
            self._lease_manager._robot_state_client,
            self._logger,
            rates.get("robot_state", 1.0),
            callbacks.get("robot_state", lambda: None),
        )
        self._is_connected = True
        return True

    @property
    def robot_state(self) -> robot_state_pb2:
        return self._robot_state_task.proto

    @property
    def kinematic_state(self) -> robot_state_pb2.KinematicState:
        if self._robot_state_task.proto is None:
            return None
        return self._robot_state_task.proto.kinematic_state

    @property
    def arm_state(self) -> robot_state_pb2.ManipulatorState:
        if self._robot_state_task.proto is None:
            return None
        return self._robot_state_task.proto.manipulator_state

    @property
    def hand_force(self):
        state = self.arm_state
        if state is None:
            return None
        ee_force = state.manipulator_state.estimated_end_effector_force_in_hand
        return ee_force.x, ee_force.y, ee_force.z

    @property
    def robot_time(self) -> timestamp_pb2.Timestamp:
        return self._lease_manager.robot.time_sync.robot_timestamp_from_local_secs(
            time.time()
        )

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
            self._lease_manager.logger.info(
                "Robot is not powered on. Attempting to power on."
            )
            self._lease_manager.robot.power_on(timeout_sec=20)
            assert self._lease_manager.robot.is_powered_on(), "Robot power on failed."
            self._lease_manager.logger.info("Robot powered on.")
        else:
            self._lease_manager.robot.logger.info("Verified that robot is powered on.")

        self.verify_estop()

    def generate_se2trajectory(self, ros_logger, wbc_points, frame_name, times, wbc_cmd):

        # body_cmd = robot_command_pb2.RobotCommand()
        for point, traj_time in zip(wbc_points, times):
           wbc_cmd_point = wbc_cmd.synchronized_command.mobility_command.se2_trajectory_request.trajectory.points.add()
           # Assign spatial parameters
           wbc_cmd_point.pose.position.x = point[0]
           wbc_cmd_point.pose.position.y = point[1]
           wbc_cmd_point.pose.angle = point[2]
           # Assign time
           duration = seconds_to_duration(traj_time)
           wbc_cmd_point.time_since_reference.CopyFrom(duration)
        # Specify frame name
        wbc_cmd.synchronized_command.mobility_command.se2_trajectory_request.se2_frame_name = frame_name
        # Apply mobility constraints
        speed_limit = SE2VelocityLimit(max_vel=SE2Velocity(linear=Vec2(x=2, y=2), angular=2),
                                       min_vel=SE2Velocity(linear=Vec2(x=-2, y=-2), angular=-2))
        mobility_params = spot_command_pb2.MobilityParams(vel_limit=speed_limit)

        wbc_cmd.synchronized_command.mobility_command.params.CopyFrom(
            RobotCommandBuilder._to_any(mobility_params))
        return wbc_cmd



    # Execute long trajectories
    def wbc_long_trajectory_executor(
        self, traj_point_positions, traj_point_velocities, timepoints, ros_logger
    ):
        ros_logger.info(f"Inside WBC trajectory executor")

        # self._lease_manager.robot.time_sync.wait_for_sync()
        # self.verify_power_and_estop()
        self._lease_manager.robot.logger.info("Robot body and arm about to move.")

        # Get to the start configuration of the trajectory before we execute it
        start_time = time.time()
        ref_time = seconds_to_timestamp(start_time)
        TRAJ_APPROACH_TIME = 1.0
        # END_TIME = 1.0

        # ros_logger.info(f"Traj point positions [0]: \n {traj_point_positions[0]}")
        body_positions = traj_point_positions[0][:3] # first 3 are body
        arm_positions = traj_point_positions[0][-6:] #last six elements are arm
        # ros_logger.info(f"Here are body positions: \n {body_positions}")
        # ros_logger.info(f"Here are arm positions: \n {arm_positions}")
        arm_cmd = RobotCommandBuilder.arm_joint_move_helper(
            joint_positions=[arm_positions],
            # joint_velocities=[traj_point_velocities[0]],
            times=[TRAJ_APPROACH_TIME],
            ref_time=ref_time,
            max_acc=10000,
            max_vel=10000,
        )

        ros_logger.info(f"arm cmd before wbc: \n {arm_cmd}")
        # synchro_arm_cmd = RobotCommandBuilder.build_synchro_command(arm_cmd)
        wbc_cmd = self.generate_se2trajectory(ros_logger, wbc_points=[body_positions],frame_name="odom", times = [TRAJ_APPROACH_TIME], wbc_cmd=arm_cmd)
        # robot_cmd = RobotCommandBuilder.build_synchro_command(body_cmd, arm_cmd)
        robot_cmd = wbc_cmd
        ros_logger.info(f"arm cmd: \n {arm_cmd}")
        ros_logger.info(f"WBC cmd: \n {wbc_cmd}")
        # ros_logger.info(f"WBC cmd: \n {robot_cmd}")
        # ros_logger.info(f"WBC cmd id: \n {robot_cmd}")

        # Send command to the robot
        # self._lease_manager.robot_command(robot_cmd)
        # self._lease_manager.robot_command(robot_cmd, time.time()+TRAJ_APPROACH_TIME)

        traj_index = [0, 9]
        end_index = len(traj_point_positions)

        # Compute reference time for the entire long trajectory
        start_time = time.time() + TRAJ_APPROACH_TIME
        ref_time = seconds_to_timestamp(start_time)

        # while traj_index[0] < end_index:
        while traj_index[0] < 2:
            times = []
            positions = []
            velocities = []

            # Don't let the extracted index range go beyond the end of the trajectory
            if traj_index[1] > end_index:
                traj_index[1] = end_index

            # Extract a short trajectory from the long list
            times = timepoints[traj_index[0] : traj_index[1]]
            positions = traj_point_positions[traj_index[0] : traj_index[1]]# need to extract arm portion
            # velocities = traj_point_velocities[traj_index[0] : traj_index[1]]
            body_positions = [sublist[:3] for sublist in positions] # first 3 are body
            arm_positions = [sublist[-6:] for sublist in positions] #last six elements are arm
            # ros_logger.info(f"Here are body positions: \n {body_positions}")
            # ros_logger.info(f"Here are arm positions: \n {arm_positions}")

            # Increment indices for the next short trajectory
            traj_index = list(map(lambda x: x + 9, traj_index))

            # Generate arm command
            arm_cmd = RobotCommandBuilder.arm_joint_move_helper(
                joint_positions=arm_positions,
                # joint_velocities=velocities,
                times=times,
                ref_time=ref_time,
                max_acc=10000,
                max_vel=10000,
            )

            # Generate body command
            # body_cmd = self.generate_se2trajectory(ros_logger, wbc_points=body_positions,frame_name="odom", times = times, body_cmd=synchro_cmd_temp)
            # self._lease_manager.robot.logger.info("Here is the body command: \n", body_cmd)
            # self._lease_manager.robot.logger.info("Here is the arm command: \n", arm_cmd)

            # Build synchro command
            # robot_cmd = RobotCommandBuilder.build_synchro_command(arm_cmd, body_cmd)
            # robot_cmd = RobotCommandBuilder.build_synchro_command(mobility_request=body_cmd, arm_request=arm_cmd)
            # synchro_arm_cmd = RobotCommandBuilder.build_synchro_command(arm_cmd)
            wbc_cmd = self.generate_se2trajectory(ros_logger, wbc_points=body_positions,frame_name="odom", times = times, wbc_cmd=arm_cmd)
            robot_cmd = wbc_cmd
            # robot_cmd = RobotCommandBuilder.build_synchro_command(body_cmd, arm_cmd)
            # ros_logger.info(f"arm cmd: \n {arm_cmd}")
            # ros_logger.info(f"WBC cmd: \n {wbc_cmd}")
            # ros_logger.info(f"End time: \n {times[-1]}")

            # break

            # Compute sleep time and sleep before executing next trajectory
            if traj_index[0] > 9:
                time.sleep(time_to_goal_in_seconds - (time.time() - time_index) - 0.05)

            # success, msg, cmd_id = self._lease_manager.robot_command(robot_cmd)
            # success, msg, cmd_id = self._lease_manager.robot_command(robot_cmd, time.time()+times[-1])
            # ros_logger.info(f"Command success: {success}")

            time_index = time.time()
            feedback_resp = self._lease_manager.robot_command_feedback(cmd_id)
            joint_move_feedback = (
                feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_joint_move_feedback
            )
            time_to_goal: duration_pb2.Duration = joint_move_feedback.time_to_goal
            time_to_goal_in_seconds: float = time_to_goal.seconds + (
                float(time_to_goal.nanos) / float(10 ** 9)
            )

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
            # joint_velocities=[traj_point_velocities[0]],
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
            # velocities = traj_point_velocities[traj_index[0] : traj_index[1]]

            # Increment indices for the next short trajectory
            traj_index = list(map(lambda x: x + 9, traj_index))

            robot_cmd = RobotCommandBuilder.arm_joint_move_helper(
                joint_positions=positions,
                # joint_velocities=velocities,
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

    def ee_velocity_msg_executor(
        self, request: arm_command_pb2.ArmVelocityCommand.Request
    ) -> Tuple[bool, Text]:

        arm_command = arm_command_pb2.ArmCommand.Request(arm_velocity_command=request)
        synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
            arm_command=arm_command
        )
        robot_cmd = robot_command_pb2.RobotCommand(
            synchronized_command=synchronized_command
        )

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
        return True, "Robot standing"

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
            return False, Text("Could not set gripper angle to invalid angle" + angle)

        # The open angle command does not take degrees but the limits
        # defined in the urdf, that is why we have to interpolate
        closed = 0.349066
        opened = -1.396263
        angle = angle / 90.0 * (opened - closed) + closed

        (
            success,
            msg,
            id,
        ) = robot_cmd = RobotCommandBuilder.claw_gripper_open_angle_command(angle)
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
