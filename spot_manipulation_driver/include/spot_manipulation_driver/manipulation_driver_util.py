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
from bosdyn.api import (basic_command_pb2, estop_pb2, geometry_pb2,
                        mobility_command_pb2, robot_command_pb2,
                        synchronized_command_pb2)
from bosdyn.client import ResponseError, RpcError
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease import (InvalidResourceError, LeaseKeepAlive,
                                 NotAuthoritativeServiceError,
                                 ResourceAlreadyClaimedError)
from bosdyn.client.robot_command import (RobotCommandBuilder,
                                         RobotCommandClient,
                                         block_until_arm_arrives,
                                         blocking_stand)
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.docking import DockingClient, blocking_dock_robot, blocking_undock
from bosdyn.util import seconds_to_timestamp
from google.protobuf.timestamp_pb2 import Timestamp
from bosdyn.client import frame_helpers
from bosdyn.client import math_helpers
from bosdyn.api import image_pb2
from bosdyn.client.image import (
    ImageClient,
    build_image_request,
    UnsupportedPixelFormatRequestedError
)

hand_image_sources = [
    "hand_image",
    # "hand_depth",
    "hand_color_image",
    # "hand_depth_in_hand_color_frame",
]

class SpotManipulationDriver(object):

    # Robot Object
    robot = None

    # Clients
    command_client = None
    robot_state_client = None
    estop_client = None
    lease_client = None
    image_client = None

    # Other attributes
    robot_state = None
    lease = None

    # Constructor
    def __init__(self):
        assert self.robot.has_arm(), "Robot requires arm to use SpotManipulationDriver"

        

    # Authenticate robot
    def authenticate_robot(self, argv):
        # parse arguments
        parser = argparse.ArgumentParser()
        bosdyn.client.util.add_base_arguments(parser)
        config = parser.parse_args(argv)

        # Setup logging
        bosdyn.client.util.setup_logging(config.verbose)
        # create robot and authenticate
        sdk = bosdyn.client.create_standard_sdk("SpotManipulationDriverClient")
        self.robot = sdk.create_robot(config.hostname)

        self.robot.authenticate("user", "b7bih964c5qg")

        # bosdyn.client.util.authenticate(self.robot)

    # Initialize clients
    def init_clients(self):
        self.estop_client = self.robot.ensure_client(EstopClient.default_service_name)
        self.command_client = self.robot.ensure_client(
            RobotCommandClient.default_service_name
        )
        self.lease_client = self.robot.ensure_client(
            bosdyn.client.lease.LeaseClient.default_service_name
        )
        self.robot_state_client = self.robot.ensure_client(
            RobotStateClient.default_service_name
        )
        self.image_client = self.robot.ensure_client(
            ImageClient.default_service_name
        )
        self.hand_image_requests = []
        for source in hand_image_sources:
            self.hand_image_requests.append(
                build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW)
            )
        
        self.docking_client = self.robot.ensure_client(
            DockingClient.default_service_name
        )

    # Useful getters
    def get_robot(self):
        return self.robot

    def get_robot_state(self):
        return self.robot_state_client.get_robot_state()

    def get_joint_states(self):

        joint_states_name = []
        joint_states_position = []
        joint_states_velocity = []
        joint_states_effort = []

        # Get robot time as local time
        state = self.get_robot_state()
        local_time = self.get_robot_time_as_local_time(
            state.kinematic_state.acquisition_timestamp
        )
        # Pack joint states into returnable variables
        for joint in state.kinematic_state.joint_states:
            if joint.name == "arm0.hr0":  # Ignore this joint
                continue
            joint_states_name.append(
                self.joint_names_remapping_dict().get(joint.name, "ERROR")
            )
            joint_states_position.append(joint.position.value)
            joint_states_velocity.append(joint.velocity.value)
            joint_states_effort.append(joint.load.value)
        return (
            local_time,
            joint_states_name,
            joint_states_position,
            joint_states_velocity,
            joint_states_effort,
        )

    def get_estimated_eef_force_state(self):
        state = self.get_robot_state()
        ee_force = state.manipulator_state.estimated_end_effector_force_in_hand
        return ee_force
        # TODO: Check values  
        # Forces with the arm in stow pose: 
        # x: 14.5269775390625
        # y: -1.1439926624298096
        # z: 13.629752159118652

    # Remap sdk joint names to easy-to-read names
    def joint_names_remapping_dict(self):
        with open(
            # "./../../config/joint_names.yaml", "r"
            "/home/spot/catkin_ws/src/nrg_spot_manipulation/spot_manipulation_driver/config/joint_names.yaml",
            "r",
        ) as old_joint_names:  # TODO: Use relative path
            try:
                new_joint_names = yaml.safe_load(old_joint_names)
                return new_joint_names
            except yaml.YAMLError as exc:
                print(exc)

    # Translate robot time to local time: Function borrowed from heuristicus/spot-ros
    def get_robot_time_as_local_time(self, timestamp):
        self.robot.time_sync.wait_for_sync()
        rtime = Timestamp()
        time_skew = self.robot.time_sync.endpoint.clock_skew
        rtime.seconds = timestamp.seconds - time_skew.seconds
        rtime.nanos = timestamp.nanos - time_skew.nanos
        if rtime.nanos < 0:
            rtime.nanos = rtime.nanos + 1000000000
            rtime.seconds = rtime.seconds - 1

        # Workaround for timestamps being incomplete
        if rtime.seconds < 0:
            rtime.seconds = 0
            rtime.nanos = 0

        return rtime

    # Convert trajectory message from ROS MoveIt or other platforms to needed style
    def convert_ros_trajectory_msg(self, msg):
        traj_point_positions = []
        traj_point_velocities = []
        time_since_ref = []

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
        for i in range(0, len(msg.trajectory.points)):
            pos_dict = {}
            vel_dict = {}

            for j in range(0, 6):
                pos_dict[msg.trajectory.joint_names[j]] = msg.trajectory.points[
                    i
                ].positions[j]
                vel_dict[msg.trajectory.joint_names[j]] = msg.trajectory.points[
                    i
                ].velocities[j]

            traj_point_positions.append(
                list(map(lambda joint_name: pos_dict[joint_name], joint_order))
            )
            traj_point_velocities.append(
                list(map(lambda joint_name: vel_dict[joint_name], joint_order))
            )
            time_since_ref.append(msg.trajectory.points[i].time_from_start.to_sec())

        return traj_point_positions, traj_point_velocities, time_since_ref

    # Verify that an e-stop exists: function borrowed from arm_joint_long_trajectory example
    def verify_estop(self):

        if self.estop_client.get_status().stop_level != estop_pb2.ESTOP_LEVEL_NONE:
            error_message = (
                "Robot is estopped. Please use an external E-Stop client, such as the"
                " estop SDK example, to configure E-Stop."
            )
            self.robot.logger.error(error_message)
            raise Exception(error_message)

    # Verify that an e-stop exists: function borrowed from arm_joint_long_trajectory example
    def verify_power_and_estop(self):
        if not self.robot.is_powered_on():
            self.robot.logger.info("Robot is not powered on. Attempting to power on.")
            self.robot.power_on(timeout_sec=20)
            assert self.robot.is_powered_on(), "Robot power on failed."
            self.robot.logger.info("Robot powered on.")
        else:
            self.robot.logger.info("Verified that robot is powered on.")

        if self.estop_client.get_status().stop_level != estop_pb2.ESTOP_LEVEL_NONE:
            error_message = (
                "Robot is estopped or estop is not set up. Please configure an e-stop."
            )
            self.robot.logger.error(error_message)
            raise Exception(error_message)

    def check_is_powered_on(self):
        # """Determine if the robot is powered on or off."""
        # power_state = self.robot_state_client.get_robot_state().power_state
        # self.powered_on = power_state.motor_power_state == power_state.STATE_ON
        # return self.powered_on

        return self.robot.is_powered_on()

    # Execute long trajectories
    def arm_long_trajectory_executor(
        self, traj_point_positions, traj_point_velocities, time_since_ref
    ):

        self.robot.time_sync.wait_for_sync()
        self.verify_power_and_estop()
        self.robot.logger.info("Arm is about to move.")

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

        self.command_client.robot_command(robot_cmd)

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
                print("time_to_goal_in_seconds" , time_to_goal_in_seconds)

            cmd_id = self.command_client.robot_command(robot_cmd)

            time_index = time.time()
            feedback_resp = self.command_client.robot_command_feedback(cmd_id)
            joint_move_feedback = (
                feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_joint_move_feedback
            )
            time_to_goal: Duration = joint_move_feedback.time_to_goal
            time_to_goal_in_seconds: float = time_to_goal.seconds + (
                float(time_to_goal.nanos) / float(10 ** 9)
            )

    def gripper_trajectory_executor(self, traj_point_positions, time_since_ref):

        self.robot.time_sync.wait_for_sync()

        self.verify_power_and_estop()

        self.robot.logger.info("Gripper is about to move.")

        # Retrieve endpoint of the trajectory and execute it
        end_index = len(traj_point_positions) - 1
        position = traj_point_positions[end_index]
        robot_cmd = RobotCommandBuilder.claw_gripper_open_angle_command(position)
        self.command_client.robot_command(robot_cmd)
        time.sleep(time_since_ref[end_index])

    def gripper_trajectory_executor_with_time_control(
        self, traj_point_positions, time_since_ref
    ):
        self.robot.time_sync.wait_for_sync()

        self.verify_power_and_estop()

        self.robot.logger.info("Gripper is about to move.")

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
        # Consget_robot_statetraints and scale
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

        end_effector_velocity = bosdyn.api.arm_command_pb2.ArmVelocityCommand.CartesianVelocity(
            frame_name="body", velocity_in_frame_name=linear
        )
        arm_velocity_command = bosdyn.api.arm_command_pb2.ArmVelocityCommand.Request(
            cartesian_velocity=end_effector_velocity,
            angular_velocity_of_hand_rt_odom_in_hand=angular,
            end_time=end_time
        )
        arm_command = bosdyn.api.arm_command_pb2.ArmCommand.Request(
            arm_velocity_command=arm_velocity_command
        )
        synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
            arm_command=arm_command
        )
        robot_cmd = robot_command_pb2.RobotCommand(
            synchronized_command=synchronized_command
        )

        self.command_client.robot_command(robot_cmd)


    def walk_robot(self, msg):
        linear = geometry_pb2.Vec2(x=msg.linear.x, y=msg.linear.y)
        angular = msg.angular.z
        end_time = seconds_to_timestamp(time.time() + 0.1)
        velocity = geometry_pb2.SE2Velocity(linear=linear, angular=angular)
        se2_vel_cmd = basic_command_pb2.SE2VelocityCommand.Request(
            end_time=end_time, se2_frame_name="body", velocity=velocity
        )
        mobility_command = mobility_command_pb2.MobilityCommand.Request(
            se2_velocity_request=se2_vel_cmd
        )
        synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
            mobility_command=mobility_command
        )
        robot_cmd = robot_command_pb2.RobotCommand(
            synchronized_command=synchronized_command
        )

        self.command_client.robot_command(robot_cmd)

    def trajectory_cmd(self,
        goal_x,
        goal_y,
        goal_heading,
        cmd_duration,
        frame_name="odom",        
        mobility_params=None
    ):
        """Send a trajectory motion command to the robot.

        Args:
            goal_x: Position X coordinate in meters
            goal_y: Position Y coordinate in meters
            goal_heading: Pose heading in radians
            cmd_duration: Time-to-live for the command in seconds.
            frame_name: frame_name to be used to calc the target position. 'odom' or 'vision'
            precise_position: if set to false, the status STATUS_NEAR_GOAL and STATUS_AT_GOAL will be equivalent. If
            true, the robot must complete its final positioning before it will be considered to have successfully
            reached the goal.

        Returns: (bool, str) tuple indicating whether the command was successfully sent, and a message
        """
        if mobility_params is None:
            mobility_params = RobotCommandBuilder.mobility_params()
        # self._trajectory_status_unknown = False
        # self._at_goal = False
        # self._near_goal = False
        # self._last_trajectory_command_precise = precise_position
        self.robot.logger.info("got command duration of {}".format(cmd_duration))
        end_time = time.time() + cmd_duration
        if frame_name == "vision":
            vision_tform_body = frame_helpers.get_vision_tform_body(
                self.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
            )
            body_tform_goal = math_helpers.SE3Pose(
                x=goal_x, y=goal_y, z=0, rot=math_helpers.Quat.from_yaw(goal_heading)
            )
            vision_tform_goal = vision_tform_body * body_tform_goal
            response = self.command_client.robot_command(
                RobotCommandBuilder.synchro_se2_trajectory_point_command(
                    goal_x=vision_tform_goal.x,
                    goal_y=vision_tform_goal.y,
                    goal_heading=vision_tform_goal.rot.to_yaw(),
                    frame_name=frame_helpers.VISION_FRAME_NAME,
                    params=mobility_params,
                ),
                end_time_secs=end_time,
            )
        elif frame_name == "odom":
            odom_tform_body = frame_helpers.get_odom_tform_body(
                self.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
            )
            body_tform_goal = math_helpers.SE3Pose(
                x=goal_x, y=goal_y, z=0, rot=math_helpers.Quat.from_yaw(goal_heading)
            )
            odom_tform_goal = odom_tform_body * body_tform_goal
            response = self.command_client.robot_command(
                RobotCommandBuilder.synchro_se2_trajectory_point_command(
                    goal_x=odom_tform_goal.x,
                    goal_y=odom_tform_goal.y,
                    goal_heading=odom_tform_goal.rot.to_yaw(),
                    frame_name=frame_helpers.ODOM_FRAME_NAME,
                    params=mobility_params,
                ),
                end_time_secs=end_time,
            )
        else:
            raise ValueError("frame_name must be 'vision' or 'odom'")
        # if response[0]:
        #     self._last_trajectory_command = response[2]
        return True, "Success"

    def dock(self, dock_id):
        """Dock the robot to the docking station with fiducial ID [dock_id]."""
        try:
            # Make sure we're powered on and standing
            self.robot.power_on()
            self.stand_robot()
            # Dock the robot
            # self._last_docking_command = dock_id
            blocking_dock_robot(self.robot, dock_id)
            # self._last_docking_command = None
            # Necessary to reset this as docking often causes the last stand command to go into an unknown state
            # self._last_stand_command = None
            return True, "Success"
        except Exception as e:
            return False, f"Exception while trying to dock: {e}"

    def undock(self, timeout=20):
        """Power motors on and undock the robot from the station."""
        try:
            # Maker sure we're powered on
            self.robot.power_on()
            # Undock the robot
            blocking_undock(self.robot, timeout)
            return True, "Success"
        except Exception as e:
            return False, f"Exception while trying to undock: {e}"

    def stand_robot(self):
        self.robot.logger.info("Commanding robot to stand...")
        # blocking_stand(self.command_client, timeout_sec=10)
        # self.robot.logger.info("Robot standing.")

        try:
            robot_cmd = RobotCommandBuilder.synchro_stand_command()
            cmd_id = self.command_client.robot_command(robot_cmd)
            self.robot.logger.info("Robot standing.")
        except Exception as e:
            self.robot.logger.error(f"Unable to execute robot command: {e}")
            return False, str(e)
        return True, "Success"
        
    def sit_robot(self):
        try:
            robot_cmd = RobotCommandBuilder.synchro_sit_command()
            cmd_id = self.command_client.robot_command(robot_cmd)
        except Exception as e:
            self.robot.logger.error(f"Unable to execute robot command: {e}")
            return False, str(e)
        return True, "Success"

    def stow_arm(self):
        try:
            robot_cmd = RobotCommandBuilder.arm_stow_command()
            cmd_id = self.command_client.robot_command(robot_cmd)
            block_until_arm_arrives(self.command_client, cmd_id)
        except Exception as e:
            self.robot.logger.error(f"Unable to execute robot command: {e}")
            return False, str(e)
        return True, "Success"

    def unstow_arm(self):
        try:
            robot_cmd = RobotCommandBuilder.arm_ready_command()
            cmd_id = self.command_client.robot_command(robot_cmd)
            time.sleep(2.0)
        except Exception as e:
            self.robot.logger.error(f"Unable to execute robot command: {e}")
            return False, str(e)
        return True, "Success"

    def open_gripper(self):
        try:
            robot_cmd = RobotCommandBuilder.claw_gripper_open_command()
            cmd_id = self.command_client.robot_command(robot_cmd)
            self.robot.logger.info("Command gripper open sent")
            time.sleep(2.0)
        except Exception as e:
            self.robot.logger.error(f"Unable to execute robot command: {e}")
            return False, str(e)
        return True, "Success"

    def close_gripper(self):
        try:
            robot_cmd = RobotCommandBuilder.claw_gripper_close_command()
            cmd_id = self.command_client.robot_command(robot_cmd)
            self.robot.logger.info("Command gripper close sent")
            time.sleep(2.0)
        except Exception as e:
            self.robot.logger.error(f"Unable to execute robot command: {e}")
            return False, str(e)
        return True, "Success"

    def gripper_angle_open(self, gripper_ang):
        # takes an angle between 0 (closed) and 90 (fully opened) and opens the
        # gripper at this angle
        if gripper_ang > 90 or gripper_ang < 0:
            return False, "Gripper angle must be between 0 and 90"
        try:        
            # The open angle command does not take degrees but the limits
            # defined in the urdf, that is why we have to interpolate
            closed = 0.349066
            opened = -1.396263
            angle = gripper_ang / 90.0 * (opened - closed) + closed
            robot_cmd = RobotCommandBuilder.claw_gripper_open_angle_command(angle)

            # Command issue with RobotCommandClient
            cmd_id = self.command_client.robot_command(robot_cmd)
            self.robot.logger.info("Command gripper open angle sent")
            time.sleep(2.0)
        except Exception as e:
            return False, f"Exception occured while gripper was moving: {e}"

        return True, "Opened gripper successfully"

    def get_hand_images(self):
        try:
            return self.image_client.get_image(self.hand_image_requests)
            # return self.image_client.get_image_async(self.hand_image_requests)            
        except UnsupportedPixelFormatRequestedError as e:
            return None

    def safe_power_off(self):
        self.robot.power_off(cut_immediately=False, timeout_sec=20)
        assert not self.robot.is_powered_on(), "Robot power off failed."
        self.robot.logger.info("Robot safely powered off.")
    
    def estop_hard(self):
        try:
            self.estop_keepalive.stop()
            return True, "Success"
        except Exception as e:
            return False, f"Exception while attempting to estop: {e}"

    def estop_soft(self):
        try:
            self.estop_keepalive.settle_then_cut()
            return True, "Success"
        except Exception as e:
            return False, f"Exception while attempting to estop: {e}"

    def disengage_estop(self):
        try:
            self.estop_keepalive.allow()
            return True, "Success"
        except Exception as e:
            return False, f"Exception while attempting to disengage estop {e}"

    def forceGetLease(self):
        try:
            self.lease = self.lease_client.acquire()
        except (InvalidResourceError, NotAuthoritativeServiceError) as err:
            return False, err.error_message

        self.lease_keepalive = LeaseKeepAlive(self.lease_client, must_acquire=True)

    def forceClaim(self) -> Tuple[bool, Text]:
        """Get a lease for the robot, a handle on the estop endpoint, and the ID of the robot."""
        try:
            self.resetEStop()
            if not self.forceGetLease():
                return False
            self.resetEStop()
        except (ResponseError, RpcError) as err:
            return False, err.error_message

        return True, "Success"

    # Functions that follow have been borrowed from NRG's ROS2 Spot wrapper. TODO: Inherit them from the SpotWrapper class

    def getLease(self) -> Tuple[bool, Text]:
        """Get a lease for the robot and keep the lease alive automatically."""
        try:
            self.lease = self.lease_client.acquire()
        except (
            ResourceAlreadyClaimedError,
            InvalidResourceError,
            NotAuthoritativeServiceError,
        ) as err:
            return False, err.error_message

        self.lease_keepalive = LeaseKeepAlive(self.lease_client, must_acquire=True)

        return True, "Success"

    def claim(self) -> Tuple[bool, Text]:
        """Get a lease for the robot, a handle on the estop endpoint, and the ID of the robot."""
        try:
            if not self.getLease():
                return False
            self.resetEStop()
        except (ResponseError, RpcError) as err:
            return False, err.error_message

        return True, "Success"

    def resetEStop(self) -> None:
        """Get keepalive for eStop"""
        self.estop_endpoint = EstopEndpoint(self.estop_client, "ros", 9.0)
        self.estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.
        self.estop_keepalive = EstopKeepAlive(self.estop_endpoint)

    def releaseLease(self) -> None:
        """Return the lease on the body."""
        if self.lease:
            self.lease_client.return_lease(self.lease)
            self.lease = None

    def release(self) -> bool:
        """Return the lease on the body and the eStop handle."""
        try:
            self.releaseLease()
            self.releaseEStop()
        except Exception as err:
            return False, Text(err)

        return True, "Success"

    def disconnect(self) -> None:
        """Release control of robot as gracefully as posssible."""
        if self.robot is None:
            return

        if self.robot.time_sync:
            self.robot.time_sync.stop()
        if self.check_is_powered_on():
            self.robot.logger.info("Robot is powered on. Attempting to stow the arm and power off")
            self.stow_arm()
            self.safe_power_off()
                
        self.release()

    def releaseEStop(self) -> None:
        """Stop eStop keepalive"""
        if self.estop_keepalive:
            self.estop_keepalive.stop()
            self.estop_keepalive = None
            self.estop_endpoint = None
