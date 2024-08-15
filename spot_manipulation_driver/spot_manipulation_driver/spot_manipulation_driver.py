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
import threading
from typing import Text, Tuple, List

from bosdyn.api import (arm_command_pb2, estop_pb2, image_pb2,
                        robot_command_pb2, robot_state_pb2, trajectory_pb2,
                        synchronized_command_pb2, basic_command_pb2, geometry_pb2, manipulation_api_pb2)
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api.spot.robot_command_pb2 import BodyControlParams
from bosdyn.api.robot_command_pb2 import RobotCommandFeedbackResponse
from bosdyn.api.mobility_command_pb2 import MobilityCommand
from bosdyn.api.arm_command_pb2 import ArmCommand, ArmJointMoveCommand
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.frame_helpers import (ODOM_FRAME_NAME, GROUND_PLANE_FRAME_NAME, HAND_FRAME_NAME,
                                        GRAV_ALIGNED_BODY_FRAME_NAME, get_a_tform_b)
from bosdyn.client.math_helpers import SE3Pose
from bosdyn.client.robot_command import (CommandFailedErrorWithFeedback,
                                         RobotCommandBuilder, TimedOutError,
                                         blocking_stand, blocking_command)
from bosdyn.client.exceptions import RpcError
from bosdyn.util import seconds_to_timestamp, seconds_to_duration
from google.protobuf import duration_pb2, timestamp_pb2
from spot_driver.async_queries import AsyncImageService, AsyncRobotState
from spot_driver.spot_lease_manager import SpotLeaseManager
from .trajectory_manager import TrajectoryManager
from bosdyn.client.manipulation_api_client import ManipulationApiClient

MAX_BODY_POSES = 100
MAX_ARM_POINTS = 10

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

    # Execute long trajectories
    def arm_long_trajectory_executor(
        self, traj_point_positions, traj_point_velocities, timepoints, cancel_event: threading.Event
    ) -> None:

        # Make sure the robot is powered on (which implicitly implies that it's estopped as well)
        try:
            if self._lease_manager.robot.is_powered_on(5):
                self._logger.info("Arm is about to move.")
            else:
                self._logger.warn("Cannot execute arm long trajectory, robot is not powered on")
                return False
        except RpcError as e:
            self._logger.warn(f"Cannot execute arm long trajectory, unable to communicate with robot\n\tMsg: {e}")
            return False

        start_time = time.time()
        ref_time = seconds_to_timestamp(start_time)

        arm_trajectory_manager = TrajectoryManager(
            points=traj_point_positions,
            times_since_ref=timepoints,
            ref_time=start_time
        )

        while not arm_trajectory_manager.done():
            if cancel_event.is_set():
                self.stop_robot()
                self._logger.info("Arm trajectory action cancelled early. Stopping robot")
                return False
            
            window, timestamps, sleep_time = arm_trajectory_manager.get_window(MAX_ARM_POINTS)

            robot_cmd = RobotCommandBuilder.arm_joint_move_helper(
                joint_positions=window,
                times=timestamps,
                ref_time=ref_time,
            )

            success, msg, _ = self._lease_manager.robot_command(robot_cmd)

            if not success:
                self._logger.warn(f"arm_long_trajectory_executor: Error executing robot command: {msg}")
                return False

            if arm_trajectory_manager.last():
                time.sleep(sleep_time)
            else:
                time.sleep(max(sleep_time - 0.2, 0))
        self._logger.info("Successfully executed arm trajectory")
        return True

    def gripper_trajectory_executor(self, traj_point_positions, time_since_ref):

        self._lease_manager.robot.time_sync.wait_for_sync()

        self.verify_power_and_estop()

        self._logger.info("Gripper is about to move.")

        # Retrieve endpoint of the trajectory and execute it
        end_index = len(traj_point_positions) - 1
        position = traj_point_positions[end_index]
        robot_cmd = RobotCommandBuilder.claw_gripper_open_angle_command(position)
        success, msg, _ = self._lease_manager.robot_command(robot_cmd)
        time.sleep(time_since_ref[end_index])

        return success

    def body_manipulation_trajectory_executor(
            self, body_trajectory: List[SE3Pose], arm_trajectory: List[List[float]], timestamps: List[float]
        ) -> bool:
        
        # Make sure the robot is powered on (which implicitly implies that it's estopped as well)
        try:
            if self._lease_manager.robot.is_powered_on(5):
                self._logger.info("Beginning execution of body manipulation trajectory")
            else:
                self._logger.warn("Cannot execute body manipulation trajectory, robot is not powered on")
                return False
        except RpcError as e:
            self._logger.warn(f"Cannot execute body manipulation trajectory, unable to communicate with robot\n\tMsg: {e}")
            return False

        # Add a small epsilon to the start time to account for small errors in the time sync
        ref_time_seconds = time.time() + 0.1
        ref_time = seconds_to_timestamp(ref_time_seconds)
        ref_time_robot = self._lease_manager.robot.time_sync.robot_timestamp_from_local_secs(ref_time_seconds)

        # # Build an SE3Trajectory out of the body poses
        se3_trajectory_points = [
            trajectory_pb2.SE3TrajectoryPoint(
                pose=pose.to_proto(), 
                time_since_reference=seconds_to_duration(timestamp)
            ) for (pose, timestamp) in zip(body_trajectory, timestamps)
        ]

        # Downsample body trajectory until within total allowable poses
        while len(se3_trajectory_points) >= MAX_BODY_POSES:
            se3_trajectory_points = se3_trajectory_points[::2]

        self._logger.info(f"SE3Trajectory has {len(se3_trajectory_points)} points")
        final_body_traj = trajectory_pb2.SE3Trajectory(points = se3_trajectory_points) #, reference_time=ref_time_robot)
        body_control_params = spot_command_pb2.BodyControlParams(
            base_offset_rt_footprint = final_body_traj,
            rotation_setting = BodyControlParams.RotationSetting.ROTATION_SETTING_OFFSET
        )
        mobility_params = spot_command_pb2.MobilityParams(body_control = body_control_params)
        body_command = RobotCommandBuilder.synchro_stand_command(params=mobility_params)

        # Downsample the arm points until within allowable range
        while len(arm_trajectory) >= MAX_ARM_POINTS:
            arm_trajectory = arm_trajectory[::2]
            timestamps = timestamps[::2]

        # Create an arm command request with joint trajectory
        robot_command = RobotCommandBuilder.arm_joint_move_helper(
            joint_positions=arm_trajectory,
            joint_velocities=None,
            times=timestamps,
            ref_time=ref_time,
            max_acc=None,
            max_vel=None,
            build_on_command=body_command
        )

        # Monitor whether both parts of the trajectory have finished
        def status_fn(response):
            done = True
            synchro_fb = response.feedback.synchronized_feedback # RobotCommandFeedbackResponse
            if synchro_fb.HasField("mobility_command_feedback"):
                mob_status = synchro_fb.mobility_command_feedback.stand_feedback.status
                done = done and mob_status == basic_command_pb2.StandCommand.Feedback.Status.STATUS_IS_STANDING
            if synchro_fb.HasField("arm_command_feedback"):
                arm_status = synchro_fb.arm_command_feedback.arm_joint_move_feedback.status
                done = done and arm_status == ArmJointMoveCommand.Feedback.Status.STATUS_COMPLETE
            return done
        
        try:
            blocking_command(
                command_client=self.lease_manager.command_client, 
                command=robot_command, 
                check_status_fn=status_fn,
                timeout_sec=timestamps[-1] + 5
            )

        except Exception as e:
            self._logger.error(f"Unknown error occured in body_manipulation_trajectory_executor: {e}")
            return False

        # Manage the trajectories for the arm and the body
        # body_trajectory_manager = TrajectoryManager(
        #     points=se3_trajectory_points, 
        #     times_since_ref=timestamps, 
        #     ref_time=ref_time_seconds
        # )

        # arm_trajectory_manager = TrajectoryManager(
        #     points=arm_trajectory,
        #     times_since_ref=timestamps,
        #     ref_time=ref_time_seconds
        # )

        # body_control_params = spot_command_pb2.BodyControlParams(
        #     base_offset_rt_footprint = trajectory_pb2.SE3Trajectory(points=se3_trajectory_points, reference_time=ref_time_robot),
        #     rotation_setting = BodyControlParams.RotationSetting.ROTATION_SETTING_OFFSET
        # )

        # body_command = RobotCommandBuilder.synchro_stand_command(
        #     params=spot_command_pb2.MobilityParams(body_control=body_control_params),
        # )

        # success_1, msg_1, _ = self.lease_manager.robot_command(body_command)

        # while not (arm_trajectory_manager.done()):

        #     arm_window, arm_stamps, arm_sleep = arm_trajectory_manager.get_window(MAX_ARM_POINTS) 
        #     # body_window, _, body_sleep = body_trajectory_manager.get_window(MAX_BODY_POSES)
        #     # segment_duration = min(arm_sleep, body_sleep)

        #     # body_control_params = spot_command_pb2.BodyControlParams(
        #     #     base_offset_rt_footprint = trajectory_pb2.SE3Trajectory(points=body_window, reference_time=ref_time_robot),
        #     #     rotation_setting = BodyControlParams.RotationSetting.ROTATION_SETTING_OFFSET
        #     # )

        #     # body_command = RobotCommandBuilder.synchro_stand_command(
        #     #     params=spot_command_pb2.MobilityParams(body_control=body_control_params),
        #     # )

        #     robot_command = RobotCommandBuilder.arm_joint_move_helper(
        #         joint_positions=arm_window,
        #         joint_velocities=None,
        #         times=arm_stamps,
        #         ref_time=ref_time,
        #         max_acc=10000,
        #         max_vel=10000,
        #         # build_on_command=body_command
        #     )

        #     success, msg, _ = self.lease_manager.robot_command(robot_command, end_time_secs=arm_sleep+0.2)
        #     if not success:
        #         self._logger.warn(f"body_manipulation_trajectory_executor: Error executing robot command: {msg}")
        #         return False

        #     if arm_trajectory_manager.last(): # and body_trajectory_manager.last():
        #         time.sleep(arm_sleep)
        #     else:
        #         time.sleep(max(arm_sleep - 0.2, 0))
                
        # self._logger.info("Trajectory complete")
        # return True

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

    def image_to_grasp(self, image, pixel_coordinates):

        # Filling out grasping request
        pick_vec = geometry_pb2.Vec2(x=pixel_coordinates[0], y=pixel_coordinates[1])
        grasp = manipulation_api_pb2.PickObjectInImage(
            pixel_xy=pick_vec,
            transforms_snapshot_for_camera=image.shot.transforms_snapshot,
            frame_name_image_sensor=image.shot.frame_name_image_sensor,
            camera_model=image.source.pinhole,
        )
        grasp.grasp_params.grasp_palm_to_fingertip = (
            0.6
        )  # might need to adjust for object size

        # Specify top-down grasp

        # Add a constraint that requests that the x-axis of the gripper is pointing in the
        # negative-z direction in the vision frame.

        # The axis on the gripper is the x-axis.
        axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=1, y=0, z=0)

        # The axis in the vision frame is the negative z-axis
        axis_to_align_with_ewrt_vision = geometry_pb2.Vec3(x=0, y=0, z=-1)

        # Add the vector constraint to our proto.
        constraint = grasp.grasp_params.allowable_orientation.add()
        constraint.vector_alignment_with_tolerance.axis_on_gripper_ewrt_gripper.CopyFrom(
            axis_on_gripper_ewrt_gripper
        )
        constraint.vector_alignment_with_tolerance.axis_to_align_with_ewrt_frame.CopyFrom(
            axis_to_align_with_ewrt_vision
        )

        # Tolerance for top-down grasp
        constraint.vector_alignment_with_tolerance.threshold_radians = 1.22

        # Specify the frame we're using.
        grasp.grasp_params.grasp_params_frame_name = frame_helpers.VISION_FRAME_NAME

        # Build the proto
        grasp_request = manipulation_api_pb2.ManipulationApiRequest(
            pick_object_in_image=grasp
        )

        # Send the request
        self._lease_manager.robot.logger.info("Sending grasp request.")
        cmd_response = self._manipulation_api_client.manipulation_api_command(
            manipulation_api_request=grasp_request
        )
        # Wait for the grasp to finish
        grasp_done = False
        failed = False
        time_start = time.time()
        while not grasp_done:
            feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
                manipulation_cmd_id=cmd_response.manipulation_cmd_id
            )

            # Send a request for feedback
            response = self._manipulation_api_client.manipulation_api_feedback_command(
                manipulation_api_feedback_request=feedback_request
            )

            current_state = response.current_state
            current_time = time.time() - time_start

            self._lease_manager.robot.logger.info(
                "Current state ({time:.1f} sec): {state}".format(
                    time=current_time,
                    state=ManipulationFeedbackState.Name(current_state),
                )
            )
            failed_states = [
                manipulation_api_pb2.MANIP_STATE_GRASP_FAILED,
                manipulation_api_pb2.MANIP_STATE_GRASP_PLANNING_NO_SOLUTION,
                manipulation_api_pb2.MANIP_STATE_GRASP_FAILED_TO_RAYCAST_INTO_MAP,
                manipulation_api_pb2.MANIP_STATE_GRASP_PLANNING_WAITING_DATA_AT_EDGE,
            ]

            failed = current_state in failed_states
            grasp_done = (
                current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED
                or failed
            )

            time.sleep(0.1)

        holding_trash = not failed

        # Move the arm to a carry position.
        self._lease_manager.robot.logger.info("Grasp succeeded.")
        self._lease_manager.robot.logger.info("Going to carry pose.")
        carry_cmd = RobotCommandBuilder.arm_carry_command()
        self._lease_manager.robot_command(carry_cmd)

        time.sleep(0.75) # Wait for the carry command to finish
        return holding_trash


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
    
    def arm_cartesian_command(
        self, cartesian_command: arm_command_pb2.ArmCartesianCommand, timeout: float | None = None
    ) -> Tuple[bool, Text, int]:
        """Command the arm end effector to move a certain offset from its current position
        
        Args:
            command - The arm cartesian command to pass to the robot 
            timeout - Float seconds after which the movement is cancelled

        Returns:
            success - Whether the command was accepted by the robot
            message - Message returned from the robot command
            command_id - Integer id used to monitor the command progress
        """

        robot_command = robot_command_pb2.RobotCommand()
        robot_command.synchronized_command.arm_command.arm_cartesian_command.CopyFrom(cartesian_command)
        end_time = time.time() + timeout if timeout is not None else None
        return self.lease_manager.robot_command(robot_command, end_time)

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
    
    def stop_robot(self) -> Tuple[bool, Text]:
        success, message, _ = self._lease_manager.robot_command(RobotCommandBuilder.stop_command())
        return success, message

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
