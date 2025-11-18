#! /usr/bin/env python3
##############################################################################
#      Title     : follow_joint_trajectory_action_server.py
#      Project   : spot_manipulation_driver
#      Created   : 01/15/2023
#      Author    : Janak Panthi (Crasun Jans)
#      Copyright : Copyright The University of Texas at Austin, 2023-2030. All
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
import math
import threading

import rclpy
import rclpy.callback_groups
import rclpy.duration
from rclpy.time import Time
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, ParameterType

import tf2_py
from tf2_ros import Buffer, TransformListener

# These imports are not unused; they allow us to call tf_buffer.transform() on these types
from tf2_geometry_msgs import PoseStamped, PointStamped

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TwistStamped
from std_srvs.srv import Trigger
from spot_msgs.msg import ManipulatorCarryState, ManipulatorStowState
from spot_msgs.srv import GripperAngleMove, InverseKinematics
from spot_msgs.action import ImageToGrasp, ArmCartesianCommand
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float32, Bool, Header
from geometry_msgs.msg import WrenchStamped, Vector3
from trajectory_msgs.msg import JointTrajectory

import spot_manipulation_driver.ros_helpers as ros_helpers
from spot_driver.ros_helpers import (JointStatesToMsg, MsgToPose, MsgToVec3, 
                                    MsgToTransform, PoseToMsg, joint_name_map_BD_to_ROS, joint_name_map_ROS_to_BD, arm_joint_names)
from spot_driver.spot_lease_manager import SpotLeaseManager
from spot_manipulation_driver.spot_manipulation_driver import SpotManipulationDriver

S_TO_NS = 1000 * 1000 * 1000

ARM_JOINT_ORDER = [
    "arm0_shoulder_yaw",
    "arm0_shoulder_pitch",
    "arm0_elbow_pitch",
    "arm0_elbow_roll",
    "arm0_wrist_pitch",
    "arm0_wrist_roll",
]

GRIPPER_JOINT_ORDER = [
    "arm0_fingers"]

WHOLE_BODY_JOINT_ORDER = [
    "body_x",
    "body_y",
    "body_or"] + ARM_JOINT_ORDER + GRIPPER_JOINT_ORDER


class SpotManipulationDriverROS(Node):
    def __init__(self):

        Node.__init__(self, "spot_manipulation_driver")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.manipulation_driver: SpotManipulationDriver = None
        self._last_gripper_velocity = TwistStamped(
            header=Header(
                frame_id="hand", 
                stamp=self.get_clock().now().to_msg()
            )
        )
        self._recorded_collision_window = [False, False, False]

        self._arm_trajectory_cancel_event = threading.Event()
        self._arm_and_finger_trajectory_cancel_event = threading.Event()
        self._mobile_manipulation_trajectory_cancel_event = threading.Event()
        self._arm_cartesian_command_cancel_event = threading.Event()

        # Declare ROS parameters
        self.declare_parameter(
            "hostname",
            "default_value",
            ParameterDescriptor(
                description="Spot computer hostname.",
                type=ParameterType.PARAMETER_STRING,
                read_only=True,
            ),
        )

        self.publish_joint_states_flag = self.declare_parameter(
            "publish_joint_states",
            False,
            ParameterDescriptor(
                description="Whether or not to publish the joint states",
                type=ParameterType.PARAMETER_BOOL,
                read_only=True,
            ),
        ).value

        self.declare_parameter(
            "rates.robot_state",
            1.0,
            ParameterDescriptor(
                description="Publish rate for robot state topics",
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=1000.0, step=0.0)
                ],
                read_only=True,
            ),
        )

        self.declare_parameter(
            "action_namespace",
            "",
            ParameterDescriptor(
                description="Namespace for the action servers. Temporary fix until remppaing is added to action servers (https://github.com/ros2/rcl/pull/1170)",
                type=ParameterType.PARAMETER_STRING,
                read_only=True
            )
        )

        self.declare_parameter(
            "data_capture_mode",
            False,
            ParameterDescriptor(
                description="Whether or not to publish received action server goals on goal topics",
                type=ParameterType.PARAMETER_BOOL,
                read_only=True,
            ),
        )

        # --- Initialize action messages --- #

        # Arm-related attributes
        self.arm_feedback = FollowJointTrajectory.Feedback()
        self.arm_result = FollowJointTrajectory.Result()

        # Finger-related attributes
        self.finger_feedback = FollowJointTrajectory.Feedback()
        self.finger_result = FollowJointTrajectory.Result()
        self.finger_feedback_publish_flag = False

        # Arm-and-finger-related attributes
        self.arm_and_finger_result = FollowJointTrajectory.Result()

        # Mobile-manipulation-related attributes
        self.mobile_manipulation_result = FollowJointTrajectory.Result()

        # Image_to_grasp-related attributes
        self.image_to_grasp_result = ImageToGrasp.Result()

        # Check if we need to capture data
        self.data_capture_mode = self.get_parameter('data_capture_mode').value

    def connect(self, lease_manager: SpotLeaseManager) -> bool:
        self.get_logger().info("Connecting manipulation driver")
        self.manipulation_driver = SpotManipulationDriver(
            self.get_logger(), self.get_parameter("hostname").value
        )

        self.get_logger().info("Setting arm state callbacks")
        self.arm_state_timer = self.create_timer(1/self.get_parameter('rates.robot_state').value, self.arm_state_callback)

        if self.manipulation_driver.connect(lease_manager):
            self.get_logger().info(f"Connected to Spot {lease_manager.ID.nickname}")
        else:
            self.get_logger().fatal("Failed to launch Spot manipulation driver")
            return False

        # Create a control group to prevent multiple callbacks from commanding motion simultaneously
        motion_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        gripper_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        # Create data publishers
        self._arm_wrench_pub = self.create_publisher(WrenchStamped        , "~/manipulator_state/wrench"                  , 10)
        self._arm_vel_pub    = self.create_publisher(TwistStamped         , "~/manipulator_state/velocity"                , 10)
        self._carry_pub      = self.create_publisher(ManipulatorCarryState, "~/manipulator_state/carry_state"             , 10)
        self._stow_pub       = self.create_publisher(ManipulatorStowState , "~/manipulator_state/stow_state"              , 10)
        self._gripper_pub    = self.create_publisher(Float32              , "~/manipulator_state/gripper_open_percentage" , 10)
        self._holding_pub    = self.create_publisher(Bool                 , "~/manipulator_state/is_gripper_carrying_item", 10)
        self._collision_pub  = self.create_publisher(Bool                 , "~/manipulator_state/is_hand_in_collision"    , 10)

        self._joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._arm_goal_pub = self.create_publisher(JointTrajectory, "/arm_controller/follow_joint_trajectory/goal", 10)
        self._mobile_manipulation_goal_pub = self.create_publisher(JointTrajectory, "/mobile_manipulation_controller/follow_joint_trajectory/goal", 10)

        # Command subscriptions
        self.ee_vel_sub = self.create_subscription(
            Twist,
            "~/cmd_vel",
            self.ee_vel_sub_callback,
            10,
            callback_group=motion_callback_group,
        )
        self.ap_ee_vel_sub = self.create_subscription(
            TwistStamped,
            "/ee_twist_cmds",
            self.ap_ee_vel_sub_callback,
            10,
            callback_group=motion_callback_group,
        )

        # Create services for arm motions
        self.create_service(Trigger, "~/claim"          , self.claim_callback   )
        self.create_service(Trigger, "~/release"        , self.release_callback )
        self.create_service(Trigger, "~/power_on"       , self.power_on_callback)
        self.create_service(Trigger, "~/unstow_arm"     , self.unstow_service_callback       , callback_group=motion_callback_group)
        self.create_service(Trigger, "~/mini_unstow_arm", self.mini_unstow_service_callback  , callback_group=motion_callback_group)
        self.create_service(Trigger, "~/stow_arm"       , self.stow_service_callback         , callback_group=motion_callback_group)
        self.create_service(Trigger, "~/close_gripper"  , self.gripper_close_service_callback, callback_group=gripper_callback_group)
        self.create_service(Trigger, "~/open_gripper"   , self.gripper_open_service_callback , callback_group=gripper_callback_group)
        self.create_service(GripperAngleMove,"~/set_gripper_angle",self.gripper_angle_service_callback, callback_group=gripper_callback_group)

        # Planning services
        self.create_service(InverseKinematics, '~/solve_ik', self.inverse_kinematics_callback)

        # Handle manual action namespace 
        action_ns = self.get_parameter('action_namespace').value

        # Initialize action servers
        self.arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            f"{action_ns}/arm_controller/follow_joint_trajectory",
            self.arm_goal_callback,
            callback_group=motion_callback_group,
            cancel_callback=self.arm_goal_cancel_callback
        )

        self.finger_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            f"{action_ns}/finger_controller/follow_joint_trajectory",
            self.finger_goal_callback,
            callback_group=gripper_callback_group,
        )

        self.arm_and_finger_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            f"{action_ns}/arm_and_finger_controller/follow_joint_trajectory",
            self.arm_and_finger_goal_callback,
            callback_group=motion_callback_group,
            cancel_callback=self.arm_and_finger_goal_cancel_callback
        )

        self.mobile_manipulation_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            f"{action_ns}/mobile_manipulation_controller/follow_joint_trajectory",
            self.mobile_manipulation_goal_callback,
            callback_group=motion_callback_group,
            cancel_callback=self.mobile_manipulation_goal_cancel_callback
        )

        self.body_manipulation_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            f"{action_ns}/body_manipulation_controller/follow_joint_trajectory",
            self.body_manipulation_callback,
            callback_group=motion_callback_group
        )

        self.image_to_grasp_action_server = ActionServer(
            self,
            ImageToGrasp,
            "image_to_grasp",
            self.image_to_grasp_goal_callback,
            callback_group=motion_callback_group,
        )

        self.hand_relative_move_action_server = ActionServer(
            self,
            ArmCartesianCommand,
            "~/arm_cartesian_command",
            self.arm_cartesian_command_callback,
            callback_group=motion_callback_group
        )

        return True
    
    def arm_goal_cancel_callback(self, cancel_request):
        self._arm_trajectory_cancel_event.set()
        return CancelResponse.ACCEPT

    def arm_goal_callback(self, goal_handle: ServerGoalHandle):
        """Callback for the /spot_arm/arm_controller/follow_joint_trajectory action server """

        # Translate message and execute trajectory while publishing feedback
        traj_point_positions, traj_point_velocities, timepoints = ros_helpers.joint_trajectory_to_lists(
            goal_handle.request.trajectory, ARM_JOINT_ORDER
        )

        # Publish the received trajectory (only once) before executing for data-capture purposes
        if self.data_capture_mode:
            def arm_goal_publisher() -> None:
                timeout = 3.0
                start_time = time.time()  

                while self._arm_goal_pub.get_subscription_count() < 1:
                    if time.time() - start_time > timeout:
                        self.get_logger().info("Timed out waiting for an arm_controller/follow_joint_trajectory/goal subscriber to be available")
                        return  
                    time.sleep(0.1)  # sleep to avoid busy-waiting
                self.get_logger().info("Publishing received joint trajectory on the arm_controller/follow_joint_trajectory/goal topic")
                self._arm_goal_pub.publish(goal_handle.request.trajectory)

            arm_goal_publisher_thread = threading.Thread(target=arm_goal_publisher)
            arm_goal_publisher_thread.start()

        trajectory_success = False

        def execute_trajectory() -> None:
            nonlocal trajectory_success
            try:
                trajectory_success = self.manipulation_driver.arm_long_trajectory_executor(
                    traj_point_positions, traj_point_velocities, timepoints, self._arm_trajectory_cancel_event
                ) 
            except Exception as e:
                self._logger.info(f"Error executing arm long trajectory: {e}")
                return

        arm_execution_thread = threading.Thread(target=execute_trajectory)
        arm_execution_thread.start()
        
        rate = self.create_rate(10.0)
        while arm_execution_thread.is_alive():
            goal_handle.publish_feedback(
                ros_helpers.get_joint_state_feedback(self.manipulation_driver)
            )
            rate.sleep()

        # Join threads
        arm_execution_thread.join()
        if self.data_capture_mode:
            arm_goal_publisher_thread.join()

        if self._arm_trajectory_cancel_event.is_set():
            goal_handle.canceled()
            self._arm_trajectory_cancel_event.clear()
        elif trajectory_success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return self.arm_result

    def finger_goal_callback(self, goal_handle):
        """Callback for the /spot_arm/finger_controller/follow_joint_trajectory action server """

        self.get_logger().info(
            "Executing goal for the /spot_arm/finger_controller/follow_joint_trajectory action server"
        )

        success = True

        # Translate message and execute trajectory while publishing feedback
        traj_point_positions = []
        time_since_ref = []

        for i in range(0, len(goal_handle.request.trajectory.points)):
            traj_point_positions.append(
                goal_handle.request.trajectory.points[i].positions[0]
            )
            time_since_ref.append(
                goal_handle.request.trajectory.points[i].time_from_start.sec
                + goal_handle.request.trajectory.points[i].time_from_start.nanosec
                * 1e-9
            )

        self.finger_feedback_publish_flag = True
        finger_feedback_thread = threading.Thread(
            target=self.finger_follow_joint_trajectory_feedback, args=(goal_handle,)
        )
        finger_feedback_thread.start()

        success = self.manipulation_driver.gripper_trajectory_executor_with_time_control(traj_point_positions, time_since_ref
        )

        self.finger_feedback_publish_flag = False
        finger_feedback_thread.join()

        if success:
            goal_handle.succeed()
            self.get_logger().info("Successfully executed finger trajectory")
        else:
            goal_handle.abort()
            self.get_logger().info("finger_controller/follow_joint_trajectory action server goal aborted")

        return self.finger_result

    def arm_and_finger_goal_cancel_callback(self, cancel_request):
        self._arm_and_finger_trajectory_cancel_event.set()
        return CancelResponse.ACCEPT

    def arm_and_finger_goal_callback(self, goal_handle: ServerGoalHandle):
        """Callback for the /spot_arm/arm_and_finger_controller/follow_joint_trajectory action server """

        # Translate message and execute trajectory while publishing feedback
        traj_point_positions, traj_point_velocities, timepoints = ros_helpers.joint_trajectory_to_lists(
            goal_handle.request.trajectory, ARM_JOINT_ORDER
        )

        gripper_traj_point_positions, _ , _= ros_helpers.joint_trajectory_to_lists(
            goal_handle.request.trajectory, GRIPPER_JOINT_ORDER
        )

        trajectory_success = False

        def execute_trajectory() -> None:
            nonlocal trajectory_success
            try:
                trajectory_success = self.manipulation_driver.arm_and_gripper_long_trajectory_executor(
                    traj_point_positions, traj_point_velocities, timepoints, gripper_traj_point_positions, self._arm_and_finger_trajectory_cancel_event
                ) 
            except Exception as e:
                self._logger.info(f"Error executing arm and finger long trajectory: {e}")
                return

        arm_and_finger_execution_thread = threading.Thread(target=execute_trajectory)
        arm_and_finger_execution_thread.start()
        
        rate = self.create_rate(10.0)
        while arm_and_finger_execution_thread.is_alive():
            goal_handle.publish_feedback(
                ros_helpers.get_joint_state_feedback(self.manipulation_driver)
            )
            rate.sleep()

        arm_and_finger_execution_thread.join()

        if self._arm_and_finger_trajectory_cancel_event.is_set():
            goal_handle.canceled()
            self._arm_and_finger_trajectory_cancel_event.clear()
        elif trajectory_success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return self.arm_and_finger_result
    
    def body_manipulation_callback(self, goal_handle: ServerGoalHandle) -> FollowJointTrajectory.Result:
        """Callback for manipulation requests with body assist"""
        self.get_logger().info(
            "Received body manipulation trajectory request with {} points and {} joints".format(
                len(goal_handle.request.trajectory.points), len(goal_handle.request.trajectory.joint_names)
            )
        )

        error_code = FollowJointTrajectory.Result.SUCCESSFUL
        error_string = "Complete"

        try:
            body_poses_in_base_footprint, joint_positions, timestamps = \
                ros_helpers.get_body_manipulation_trajectories(goal_handle.request.trajectory)
        except Exception as e:
            self.get_logger().error(f"Exception raised in body manipulation trajectory generation: {e}")
            goal_handle.abort()
            return FollowJointTrajectory.Result(error_code=-1, error_string="Unknown exception occured")
        
        # TODO: feedback
        self.get_logger().info(f"Num body poses: {len(body_poses_in_base_footprint)} | Num joint positions: {len(joint_positions)} | Num timestamps: {len(timestamps)}")
        try:
            success = self.manipulation_driver.body_manipulation_trajectory_executor(
                body_poses_in_base_footprint, joint_positions, timestamps
            )
        except Exception as e:
            self._logger.warn(f"Error executing body manipulation trajectory: {e}")
            success = False
        
        if success:
            goal_handle.succeed()
            self.get_logger().info("Body maniplulation request completed successfully")
        else:
            goal_handle.abort()
            error_code = -1
            error_string = "Exception occured"

        return FollowJointTrajectory.Result(error_code=error_code, error_string=error_string)

    def mobile_manipulation_goal_cancel_callback(self, cancel_request):
        self._mobile_manipulation_trajectory_cancel_event.set()
        return CancelResponse.ACCEPT

    def mobile_manipulation_goal_callback(self, goal_handle: ServerGoalHandle):
        """Callback for the /mobile_manipulation_controller/follow_joint_trajectory action server """

        # Translate message and execute trajectory while publishing feedback
        traj_point_positions, traj_point_velocities, timepoints = ros_helpers.joint_trajectory_to_lists(
            goal_handle.request.trajectory, WHOLE_BODY_JOINT_ORDER
        )

        # Publish the received trajectory (only once) before executing for data-capture purposes
        if self.data_capture_mode:
            def mobile_manipulation_goal_publisher() -> None:
                timeout = 3.0
                start_time = time.time()  

                while self._mobile_manipulation_goal_pub.get_subscription_count() < 1:
                    if time.time() - start_time > timeout:
                        self.get_logger().info("Timed out waiting for an mobile_manipulation_controller/follow_joint_trajectory/goal subscriber to be available")
                        return  
                    time.sleep(0.1)  # sleep to avoid busy-waiting
                self.get_logger().info("Publishing received joint trajectory on the mobile_manipulation_controller/follow_joint_trajectory/goal topic")
                self._mobile_manipulation_goal_pub.publish(goal_handle.request.trajectory)

            mobile_manipulation_goal_publisher_thread = threading.Thread(target=mobile_manipulation_goal_publisher)
            mobile_manipulation_goal_publisher_thread.start()

        trajectory_success = False

        def execute_trajectory() -> None:
            nonlocal trajectory_success
            try:
                trajectory_success = self.manipulation_driver.mobile_manipulation_long_trajectory_executor(
                    traj_point_positions, traj_point_velocities, timepoints, self._mobile_manipulation_trajectory_cancel_event
                ) 
            except Exception as e:
                self._logger.info(f"Error executing mobile manipulation long trajectory: {e}")
                return

        mobile_manipulation_execution_thread = threading.Thread(target=execute_trajectory)
        mobile_manipulation_execution_thread.start()
        
        rate = self.create_rate(10.0)
        while mobile_manipulation_execution_thread.is_alive():
            goal_handle.publish_feedback(
                ros_helpers.get_joint_state_feedback(self.manipulation_driver)
            )
            rate.sleep()

        # Join threads
        mobile_manipulation_execution_thread.join()
        if self.data_capture_mode:
            mobile_manipulation_goal_publisher_thread.join()

        if self._mobile_manipulation_trajectory_cancel_event.is_set():
            goal_handle.canceled()
            self._mobile_manipulation_trajectory_cancel_event.clear()
        elif trajectory_success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return self.mobile_manipulation_result
    
    def arm_cartesian_command_cancel_callback(self, cancel_request):
        self._arm_cartesian_command_cancel_event.set()
        return CancelResponse.ACCEPT
    
    def arm_cartesian_command_callback(self, goal_handle: ServerGoalHandle) -> ArmCartesianCommand.Result:
        """Callback for the spot_manipualtion_driver/arm_cartesian_command action server """

        try:
            robot_command = ros_helpers.cartesian_request_to_command(goal_handle.request, self.tf_buffer)
            success, message, command_id = self.manipulation_driver.arm_cartesian_command(robot_command)
            if not success:
                self._logger.warn(f"Unable to execute arm cartesian command: {message}")
                goal_handle.abort()
                return ArmCartesianCommand.Result(success=False, message=message)
        except tf2_py.LookupException as e:
            self._logger.warn(f"Transform lookup error during arm cartesian command execution: {e}")
            goal_handle.abort()
            return ArmCartesianCommand.Result(success=False, message=str(e))
        except Exception as e:
            self._logger.info(f"Unknown error executing arm cartesian command: {e}")
            goal_handle.abort()
            return ArmCartesianCommand.Result(success=False, message=str(e))
        
        rate = self.create_rate(10.0)
        response = ArmCartesianCommand.Result()
        while True:
            if self._arm_cartesian_command_cancel_event.is_set():
                self.manipulation_driver.stop_robot()
                response.success = False
                response.message = "Cartesian command cancelled early, aborting movement"
                goal_handle.canceled()
                self._arm_cartesian_command_cancel_event.clear()
                break

            feedback = self.manipulation_driver.lease_manager.robot_command_feedback(command_id)
            arm_feedback = feedback.feedback.synchronized_feedback.arm_command_feedback
            if not arm_feedback.HasField("arm_cartesian_feedback"):
                self.manipulation_driver.stop_robot()
                response.success = False
                response.message = "Feedback message was not filled out, presumably because the motion was preempted. Aborting movement"
                goal_handle.abort()
                break

            elif arm_feedback.arm_cartesian_feedback.status == ArmCartesianCommand.Feedback.STATUS_TRAJECTORY_COMPLETE:
                response.success = True
                response.message = "Arm cartesian command completed successfully"
                goal_handle.succeed()
                break

            elif arm_feedback.arm_cartesian_feedback.status == ArmCartesianCommand.Feedback.STATUS_TRAJECTORY_STALLED:
                self.manipulation_driver.stop_robot()
                response.success = False
                response.message = "Unable to complete arm cartesian command, it has been stalled"
                goal_handle.abort()
                break

            elif arm_feedback.arm_cartesian_feedback.status == ArmCartesianCommand.Feedback.STATUS_TRAJECTORY_CANCELLED:
                self.manipulation_driver.stop_robot()
                response.success = False
                response.message = "Unable to complete arm cartesian command, it has been cancelled"
                goal_handle.abort()
                break

            else:
                ros_feedback = ArmCartesianCommand.Feedback()
                ros_feedback.status = arm_feedback.arm_cartesian_feedback.status
                ros_feedback.measured_pos_distance_to_goal = arm_feedback.arm_cartesian_feedback.measured_pos_distance_to_goal
                ros_feedback.measured_rot_distance_to_goal = arm_feedback.arm_cartesian_feedback.measured_rot_distance_to_goal
                ros_feedback.measured_pos_tracking_error   = arm_feedback.arm_cartesian_feedback.measured_pos_tracking_error  
                ros_feedback.measured_rot_tracking_error   = arm_feedback.arm_cartesian_feedback.measured_rot_tracking_error  
                goal_handle.publish_feedback(ros_feedback)

            rate.sleep()

        if not response.success:
            self._logger.warn(response.message)
        return response

    def image_to_grasp_goal_callback(self, goal_handle):
        """Callback for the /image_to_grasp action server """

        self.get_logger().info(
            "Executing goal for the /image_to_grasp action server"
        )
        self.log_image_to_grasp_goal(self.get_logger(), goal_handle.request)
        self.image_to_grasp_result.success = False
        image_proto = ros_helpers.img_msg_to_proto(goal_handle.request.image, goal_handle.request.camera_info, goal_handle.request.tf_msg, self.manipulation_driver)
        self.get_logger().info(f'Frame name image sensor: {image_proto.shot.frame_name_image_sensor}')
        self.get_logger().info(f'Pinhole: {image_proto.source.pinhole}')
        self.get_logger().info(f'Transforms Snap: {image_proto.shot.transforms_snapshot}')
        self.get_logger().info(f'Pixel coordinates: {goal_handle.request.pixel_coordinates}')

        self.image_to_grasp_result.success = self.manipulation_driver.image_to_grasp(image_proto, goal_handle.request.pixel_coordinates, goal_handle.request.grasp_strategy)
        if self.image_to_grasp_result.success:
            goal_handle.succeed()
            self.get_logger().info("Successfully executed image_to_grasp goal")
        else:
            goal_handle.abort()
            self.get_logger().info("image_to_grasp action server goal aborted")
        return self.image_to_grasp_result

    def log_image_to_grasp_goal(self, logger, goal: ImageToGrasp.Goal):
        """Logs an image to grasp goal"""

        # Log image info
        logger.info(f"Image encoding: {goal.image.encoding}")
        logger.info(f"Image width: {goal.image.width}, height: {goal.image.height}")

        # Log camera_info
        logger.info(f"Camera frame_id: {goal.camera_info.header.frame_id}")
        logger.info(f"Camera K matrix: {goal.camera_info.k}")

        # Log TF tree
        logger.info(f"TFMessage contains {len(goal.tf_msg.transforms)} transforms")
        for tf in goal.tf_msg.transforms:
            trans = tf.transform.translation
            rot = tf.transform.rotation
            logger.info(
                f"  {tf.header.frame_id} -> {tf.child_frame_id} | "
                f"translation: [x={trans.x:.3f}, y={trans.y:.3f}, z={trans.z:.3f}] | "
                f"rotation (quat): [x={rot.x:.3f}, y={rot.y:.3f}, z={rot.z:.3f}, w={rot.w:.3f}]"
            )

        # Log pixel coordinates
        pixel_coords_str = ", ".join(str(p) for p in goal.pixel_coordinates)
        logger.info(f"Pixel coordinates: [{pixel_coords_str}]")

        # Log grasp strategy
        logger.info(f"Grasp strategy: {goal.grasp_strategy}")

    def finger_follow_joint_trajectory_feedback(self, goal_handle: ServerGoalHandle):
        """Feedback for finger action server"""
        # Publishes actual states of the joints
        rate = self.create_rate(frequency=2.0, clock=self.get_clock())

        while self.finger_feedback_publish_flag:
            # Get joint states
            self.finger_feedback = ros_helpers.get_joint_state_feedback(
                self.manipulation_driver
            )
            goal_handle.publish_feedback(self.finger_feedback)
            rate.sleep()

    def ee_vel_sub_callback(self, msg: Twist):
        """Callback for end effector velocity command subscriber"""
        arm_vel_request = ros_helpers.twist_to_vel_request(
            self.manipulation_driver.robot_time, msg
        )
        self.manipulation_driver.ee_velocity_msg_executor(arm_vel_request)

    def ap_ee_vel_sub_callback(self, msg: TwistStamped):
        """Callback for Affordance Primitive end effector velocity command subscriber"""
        self.ee_vel_sub_callback(msg.twist)

    def arm_state_callback(self):
        self.manipulation_driver.update_robot_state()

        if self.publish_joint_states_flag:
            joint_states = JointStatesToMsg(self.manipulation_driver.kinematic_state, self.manipulation_driver.lease_manager)
            self._joint_state_pub.publish(joint_states)

        state = self.manipulation_driver.arm_state
        if state is None:
            return

        arm_force: WrenchStamped = ros_helpers.manipulator_state_to_wrench(state, self.manipulation_driver)
        arm_vel  : TwistStamped  = ros_helpers.manipulator_state_to_twist(state, self.manipulation_driver)

        self._arm_wrench_pub.publish(arm_force)
        self._arm_vel_pub.publish(arm_vel)
        self._gripper_pub.publish(Float32(data=state.gripper_open_percentage))
        self._carry_pub.publish(ManipulatorCarryState(state=state.carry_state))
        self._stow_pub.publish(ManipulatorStowState(state=state.stow_state))
        self._holding_pub.publish(Bool(data=state.is_gripper_holding_item))

        # Calculate the collision state
        dt_ros = self.get_clock().now() - Time.from_msg(self._last_gripper_velocity.header.stamp)
        dt_sec = dt_ros.nanoseconds / S_TO_NS
        arm_acc = Vector3()
        arm_acc.x = (arm_vel.twist.linear.x - self._last_gripper_velocity.twist.linear.x)/dt_sec
        arm_acc.y = (arm_vel.twist.linear.y - self._last_gripper_velocity.twist.linear.y)/dt_sec
        arm_acc.z = (arm_vel.twist.linear.z - self._last_gripper_velocity.twist.linear.z)/dt_sec
        self._last_gripper_velocity = arm_vel

        force = state.estimated_end_effector_force_in_hand
        arm_abs_acc = math.sqrt(arm_acc.x**2 + arm_acc.y**2 + arm_acc.z**2)
        arm_abs_force = math.sqrt(force.x**2 + force.y**2 + force.z**2)

        # These values were decided through trial and error. They're probably not great
        if state.stow_state != ManipulatorStowState.STOWSTATE_DEPLOYED:
            collision_state = False
        elif arm_abs_acc < 1.0:
            collision_state = arm_abs_force > 8
        elif arm_abs_acc < 2.0:
            collision_state = arm_abs_force > 15
        else:
            collision_state = arm_abs_force > 35
        self._recorded_collision_window[1::] = self._recorded_collision_window[0:-1]
        self._recorded_collision_window[0] = collision_state

        self._collision_pub.publish(Bool(data=all(self._recorded_collision_window)))


    def claim_callback(self, _: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.claim()
        resp.success = success
        resp.message = msg
        return resp

    def release_callback(self, _: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.release()
        resp.success = success
        resp.message = msg
        return resp

    def power_on_callback(self, _: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        self.get_logger().info("Powering on...")
        (success, msg) = self.manipulation_driver.lease_manager.power_on()
        resp.success = success
        resp.message = msg
        return resp

    def stow_service_callback(self, _: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.stow_arm()
        resp.success = success
        resp.message = msg
        time.sleep(1) # sleep to ensure end config is reached
        return resp

    def unstow_service_callback(self, _: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.unstow_arm()
        resp.success = success
        resp.message = msg
        time.sleep(1) # sleep to ensure end config is reached
        return resp
    
    def mini_unstow_service_callback(self, _: Trigger.Request, resp: Trigger.Response) -> Trigger.Response :
        arm_vel_request = ros_helpers.twist_to_vel_request(self.manipulation_driver.robot_time, Twist())
        resp.success, resp.message = self.manipulation_driver.ee_velocity_msg_executor(arm_vel_request)
        time.sleep(3) # sleep to ensure end config is reached
        return resp

    def gripper_close_service_callback(self, _, resp: Trigger.Response) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.close_gripper()
        resp.success = success
        resp.message = msg
        time.sleep(0.5) # sleep to ensure end config is reached
        return resp

    def gripper_open_service_callback(self, _, resp: Trigger.Response) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.open_gripper()
        resp.success = success
        resp.message = msg
        time.sleep(0.5) # sleep to ensure end config is reached
        return resp

    def gripper_angle_service_callback(self, req: GripperAngleMove.Request, resp: GripperAngleMove.Response):
        (success, msg) = self.manipulation_driver.open_gripper_to_angle(
            req.gripper_angle
        )
        resp.success = success
        resp.message = msg
        time.sleep(0.5) # sleep to ensure end config is reached
        return resp
        
    def inverse_kinematics_callback(self, req: InverseKinematics.Request, resp: InverseKinematics.Response) -> InverseKinematics.Response:
        """ROS service handler for solving an inverse kinematics request for a tool pose or tool gaze pose"""

        resp.solution_found = False
        target_frame_id: str = 'odom'
        source_headers: list[Header] = [req.target_pose.header]
        if req.use_gaze_target: 
            source_headers.append(req.gaze_target.header)

        # Transform the tool frame into the odom frame
        for source_header in source_headers:
            if not self.tf_buffer.can_transform(target_frame_id, source_header.frame_id, Time.from_msg(source_header.stamp), rclpy.duration.Duration(seconds=1.0)):
                self.get_logger().warn(f'Unable to solve IK problem: cannot lookup transform from odom to {source_header.frame_id}')
                return resp
        
        target_pose_ros = self.tf_buffer.transform(req.target_pose, target_frame_id)
        gaze_target_ros = self.tf_buffer.transform(req.gaze_target, target_frame_id) if req.use_gaze_target else None

        # Convert the data to BD API types
        target_pose_proto = MsgToPose(target_pose_ros.pose)
        gaze_target_proto = MsgToVec3(gaze_target_ros) if req.use_gaze_target else None

        # Load the nominal joint state from the request
        if len(req.joint_names) != len(req.joint_nominal_positions):
            self.get_logger().warn(f'Error solving for IK: {len(req.joint_names)} starting position joint labels were provided, while {len(req.joint_nominal_positions)} joint positions were provided')
            return resp
        
        # Convert the joint names to the BD versions
        bd_joint_names = [joint_name_map_ROS_to_BD[name] for name in req.joint_names if name in joint_name_map_ROS_to_BD]
        nominal_joint_state=dict(zip(bd_joint_names, req.joint_nominal_positions))

        # Determine if we're using a tool attached to the wrist
        if req.tool_frame != 'arm0_hand':
            try:
                wrist_tform_tool_ros = self.tf_buffer.lookup_transform('arm0_wrist_roll', req.tool_frame, Time(), rclpy.duration.Duration(seconds=1.0))
            except tf2_py.LookupException as e:
                self.get_logger().warn(f'Unable to find tool tranform with respect to robot wrist: {e}')
                return resp
            
            wrist_tform_tool = MsgToTransform(wrist_tform_tool_ros.transform)
        else:
            wrist_tform_tool = None

        # Pass the request on to the main driver
        success, arm_joint_state, odom_tform_body = self.manipulation_driver.solve_ik(target_pose=target_pose_proto, gaze_target=gaze_target_proto, joint_state=nominal_joint_state, wrist_tform_tool=wrist_tform_tool)

        # Transform the body pose into the nominal body frame
        try:
            base_footprint_tform_odom = self.tf_buffer.lookup_transform("base_footprint", "odom", Time(seconds=0), rclpy.duration.Duration(seconds=1))
        except tf2_py.LookupException as e:
            self.get_logger().warn(f'Unable to transform body pose into base_footprint frame: {e}')
            return resp
        base_footprint_tform_body = MsgToTransform(base_footprint_tform_odom) * odom_tform_body
            
        # Populate the response
        resp.body_pose = PoseToMsg(base_footprint_tform_body)
        
        # Convert joints back to ROS versions
        resp.arm_joint_state.header.frame_id = "base_footprint"
        resp.arm_joint_state.header.stamp = self.get_clock().now().to_msg()
        resp.arm_joint_state.name = [joint_name_map_BD_to_ROS[name] for name in arm_joint_state.keys() if name in arm_joint_names]
        resp.arm_joint_state.position = [arm_joint_value for [name, arm_joint_value] in arm_joint_state.items() if name in arm_joint_names]

        resp.solution_found = success

        return resp

