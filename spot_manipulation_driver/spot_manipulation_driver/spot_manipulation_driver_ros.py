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

import threading
import time

import rclpy
import rclpy.callback_groups
import spot_manipulation_driver.ros_helpers as ros_helpers
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from rcl_interfaces.msg import (FloatingPointRange, ParameterDescriptor,
                                ParameterType)
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from spot_driver.ros_helpers import JointStatesToMsg, getImageMsg
from spot_driver.spot_lease_manager import SpotLeaseManager
from spot_manipulation_driver.spot_manipulation_driver import \
    SpotManipulationDriver
from spot_msgs.msg import ManipulatorState
from spot_msgs.srv import GripperAngleMove
from std_srvs.srv import Trigger
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R
from trajectory_msgs.msg import JointTrajectory
import numpy as np


class SpotManipulationDriverROS(Node):
    def __init__(self):

        Node.__init__(self, "spot_manipulation_driver")

        self.manipulation_driver: SpotManipulationDriver = None

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

        self.declare_parameter(
            "publish_joint_states",
            False,
            ParameterDescriptor(
                description="Whether or not to publish the joint states",
                type=ParameterType.PARAMETER_BOOL,
                read_only=True,
            ),
        )

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
            "rates.sensors.hand_image",
            1.0,
            ParameterDescriptor(
                description="Publish rate for hand images",
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=1000.0, step=0.0)
                ],
                read_only=True,
            ),
        )

        # --- Initialize action messages --- #
        # WBC-related attributes
        self.wbc_feedback = FollowJointTrajectory.Feedback()
        self.wbc_result = FollowJointTrajectory.Result()
        # self.wbc_feedback_publish_flag = False

        # Arm-related attributes
        self.arm_feedback = FollowJointTrajectory.Feedback()
        self.arm_result = FollowJointTrajectory.Result()
        self.arm_feedback_publish_flag = False

        # Finger-related attributes
        self.finger_feedback = FollowJointTrajectory.Feedback()
        self.finger_result = FollowJointTrajectory.Result()
        self.finger_feedback_publish_flag = False

        # TF parameters
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def read_transform(self, source_frame, target_frame):
        try:
            transform_stamped = self.buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
            return transform_stamped
        except TransformException as e:
            self.get_logger().warn(f"Could not get transform: {e}")


    def connect(self, lease_manager: SpotLeaseManager) -> bool:
        self.get_logger().info("Connecting manipulation driver")
        self.manipulation_driver = SpotManipulationDriver(
            self.get_logger(), self.get_parameter("hostname").value
        )

        self.get_logger().info("Setting arm state callbacks")
        callbacks = {
            "robot_state": self.arm_state_callback,
            "hand_image": self.publish_hand_images,
        }
        rates = {
            "robot_state": self.get_parameter("rates.robot_state").value,
            "hand_image": self.get_parameter("rates.sensors.hand_image").value,
        }

        publish_joint_states = self.get_parameter("publish_joint_states").value
        if publish_joint_states:
            callbacks["robot_state"] = lambda t: (
                self.publish_joint_states(t),
                self.arm_state_callback(t),
            )

        if self.manipulation_driver.connect(lease_manager, rates, callbacks):
            self.get_logger().info(f"Connected to Spot {lease_manager.ID.nickname}")
        else:
            self.get_logger().fatal("Failed to launch Spot manipulation driver")
            return False

        # Create a control group to prevent multiple callbacks from commanding motion simultaneously
        motion_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        gripper_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        # Action server goal publishers
        self.arm_goal_pub = self.create_publisher(JointTrajectory, "/arm_controller/follow_joint_trajectory/goal", 10)

        # Create data publishers and subscribers
        self._hand_image_pub = self.create_publisher(Image, "~/rgb/tof/image", 10)
        self._hand_depth_map_pub = self.create_publisher(Image, "~/depth/tof/image", 10)
        self._hand_4k_image_pub = self.create_publisher(Image, "~/rgb/camera/image", 10)
        self._hand_4k_depth_map_pub = self.create_publisher(
            Image, "~/depth/camera/image", 10
        )
        self._hand_image_info_pub = self.create_publisher(
            CameraInfo, "~/rgb/tof/camera_info", 10
        )
        self._hand_depth_map_info_pub = self.create_publisher(
            CameraInfo, "~/depth/tof/camera_info", 10
        )
        self._hand_4k_image_info_pub = self.create_publisher(
            CameraInfo, "~/rgb/camera/camera_info", 10
        )
        self._hand_4k_depth_map_info_pub = self.create_publisher(
            CameraInfo, "~/depth/camera/camera_info", 10
        )
        self._manipulator_state_pub = self.create_publisher(
            ManipulatorState, "~/manipulator_state", 10
        )

        if publish_joint_states:
            self._joint_state_pub = self.create_publisher(
                JointState, "~/joint_state", 10
            )

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
        self.create_service(Trigger, "~/claim", self.claim_callback)
        self.create_service(Trigger, "~/release", self.release_callback)
        self.create_service(Trigger, "~/power_on", self.power_on_callback)
        self.create_service(
            Trigger,
            "~/unstow_arm",
            self.unstow_service_callback,
            callback_group=motion_callback_group,
        )
        self.create_service(
            Trigger,
            "~/stow_arm",
            self.stow_service_callback,
            callback_group=motion_callback_group,
        )
        self.create_service(
            Trigger,
            "~/close_gripper",
            self.gripper_close_service_callback,
            callback_group=gripper_callback_group,
        )
        self.create_service(
            Trigger,
            "~/open_gripper",
            self.gripper_open_service_callback,
            callback_group=gripper_callback_group,
        )
        self.create_service(
            Trigger,
            "~/stand",
            self.stand_service_callback,
            callback_group=gripper_callback_group,
        )
        self.create_service(
            GripperAngleMove,
            "~/set_gripper_angle",
            self.gripper_angle_service_callback,
            callback_group=gripper_callback_group,
        )

        # Initialize action servers
        self.whole_body_control_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/body_and_arm_controller/follow_joint_trajectory",
            # "/wbc_controller/follow_joint_trajectory",
            self.whole_body_control_goal_callback,
            callback_group=motion_callback_group,
        )

        self.arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            self.arm_goal_callback,
            callback_group=motion_callback_group,
        )

        self.finger_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/finger_controller/follow_joint_trajectory",
            self.finger_goal_callback,
            callback_group=gripper_callback_group,
        )

        # Create timers to update the async tasks for publishing
        self.create_timer(
            0.5 / rates["hand_image"],
            lambda: self.manipulation_driver._hand_image_task.update(),
        )
        self.create_timer(
            0.5 / rates["robot_state"],
            lambda: self.manipulation_driver._robot_state_task.update(),
        )

        return True

    def whole_body_control_goal_callback(self, goal_handle: ServerGoalHandle):
        """Callback for the /spot_driver/wbc_controller/follow_joint_trajectory action server """

        self.get_logger().info(
            "D:Executing goal for the /spot_driver/wbc_controller/follow_joint_trajectory action server"
        )

        success = True

        # Get base_footprint to odom transform
        rosT_o2f = self.read_transform("base_footprint", "odom") # Transformation in the form of ROS transformstamped representing the pose of the base_footprint frame in odom frame
        # self.get_logger().info(f"Rotation x: {rosT_o2f.transform.rotation.x}")
        # self.get_logger().info(f"Rotation y: {rosT_o2f.transform.rotation.y}")
        # self.get_logger().info(f"Rotation z: {rosT_o2f.transform.rotation.z}")
        # self.get_logger().info(f"Rotation w: {rosT_o2f.transform.rotation.w}")
        # self.get_logger().info(f"Translation x: {rosT_o2f.transform.translation.x}")
        # self.get_logger().info(f"Translation y: {rosT_o2f.transform.translation.y}")
        # self.get_logger().info(f"Translation z: {rosT_o2f.transform.translation.z}")

        # Convert from ros-type transform to a numpy matrix
        T_o2f, yaw_o2f = ros_helpers.convert_transformstamped_to_matrix(rosT_o2f)
        # self.get_logger().info(f"Transform as np matrix: {T_o2f}")
        # self.get_logger().info(f"Yaw_o2f: {yaw_o2f}")

        # self.get_logger().info("Here is the trajectory before conversion:")
        self.get_logger().info("Here is the trajectory on ROS side:")
        self.get_logger().info(f"Joint Names: {goal_handle.request.trajectory.joint_names}")
        for point in goal_handle.request.trajectory.points:
            positions = point.positions
            point_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            self.get_logger().info(f"Time from start: {point_time}")
            self.get_logger().info(f"Joint Positions: {positions}")

        # Convert ROS message to python lists
        traj_point_positions, traj_point_velocities, timepoints = ros_helpers.wbc_joint_trajectory_to_lists(
            goal_handle.request.trajectory, T_o2f, yaw_o2f
        )
        self.get_logger().info(f"Here is the trajectory after conversion to list: {traj_point_positions}")

        # self.arm_feedback_publish_flag = True
        # arm_feedback_thread = threading.Thread(
        # target=self.arm_follow_joint_trajectory_feedback, args=(goal_handle,)
        # )
        # arm_feedback_thread.start()

        self.manipulation_driver.wbc_long_trajectory_executor(
            traj_point_positions, traj_point_velocities, timepoints, self._logger
        )

        # self.arm_feedback_publish_flag = False
        # arm_feedback_thread.join()

        if success:
            goal_handle.succeed()
            self.get_logger().info(
                "Successfully executed spot whole-body-control trajectory"
            )
        return self.wbc_result

    def arm_goal_callback(self, goal_handle: ServerGoalHandle):
        """Callback for the /spot_arm/arm_controller/follow_joint_trajectory action server """

        self.get_logger().info(
            "D:Executing goal for the /spot_arm/arm_controller/follow_joint_trajectory action server"
        )

        success = True

        # Translate message and execute trajectory while publishing feedback
        traj_point_positions, traj_point_velocities, timepoints = ros_helpers.joint_trajectory_to_lists(
            goal_handle.request.trajectory
        )

        self.arm_feedback_publish_flag = True
        arm_goal_publisher_thread = threading.Thread(target=self.arm_goal_publisher, args=(goal_handle,))
        arm_goal_publisher_thread.start()
        arm_feedback_thread = threading.Thread(
            target=self.arm_follow_joint_trajectory_feedback, args=(goal_handle,)
        )
        arm_feedback_thread.start()

        self.manipulation_driver.arm_long_trajectory_executor(
            traj_point_positions, traj_point_velocities, timepoints
        )

        self.arm_feedback_publish_flag = False
        arm_feedback_thread.join()

        if success:
            goal_handle.succeed()
            self.get_logger().info("Successfully executed arm trajectory")
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

        SpotManipulationDriver.gripper_trajectory_executor(
            self, traj_point_positions, time_since_ref
        )

        self.finger_feedback_publish_flag = False
        finger_feedback_thread.join()

        if success:
            goal_handle.succeed()
            self.get_logger().info("Successfully executed finger trajectory")
        return self.finger_result

    def arm_goal_publisher(self, goal_handle: ServerGoalHandle):
        """Feedback for arm action server"""
        # Publish the message to the goal trajectory only once
        while(self.arm_goal_pub.get_subscription_count()<1):
            self.get_logger.info("Waiting for an arm_controller/follow_joint_trajectory/goal subscriber to be available")
        self.arm_goal_pub.publish(goal_handle.request.trajectory)

    def arm_follow_joint_trajectory_feedback(self, goal_handle: ServerGoalHandle):
        """Feedback for arm action server"""
        rate = self.create_rate(frequency=2.0, clock=self.get_clock())

        # Publishes actual states of the joints
        while self.arm_feedback_publish_flag:
            self.arm_feedback = ros_helpers.get_joint_state_feedback(
                self.manipulation_driver
            )
            goal_handle.publish_feedback(self.arm_feedback)
            rate.sleep()

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

    def arm_state_callback(self, _):
        state = self.manipulation_driver.arm_state
        if state is None:
            return
        state_msg = ros_helpers.manipulator_state_to_msg(
            state, self.manipulation_driver
        )
        self._manipulator_state_pub.publish(state_msg)

    def claim_callback(
        self, _: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.claim()
        resp.success = success
        resp.message = msg
        return resp

    def release_callback(
        self, _: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.release()
        resp.success = success
        resp.message = msg
        return resp

    def power_on_callback(
        self, _: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        self.get_logger().info("Powering on...")
        (success, msg) = self.manipulation_driver.lease_manager.power_on()
        resp.success = success
        resp.message = msg
        return resp

    def stow_service_callback(
        self, _: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.stow_arm()
        resp.success = success
        resp.message = msg
        return resp

    def unstow_service_callback(
        self, _: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.unstow_arm()
        resp.success = success
        resp.message = msg
        return resp

    def gripper_close_service_callback(
        self, _, resp: Trigger.Response
    ) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.close_gripper()
        resp.success = success
        resp.message = msg
        return resp

    def gripper_open_service_callback(
        self, _, resp: Trigger.Response
    ) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.open_gripper()
        resp.success = success
        resp.message = msg
        return resp

    def stand_service_callback(self, _, resp: Trigger.Response) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.stand_robot()
        resp.success = success
        resp.message = msg
        return resp

    def gripper_angle_service_callback(
        self, req: GripperAngleMove.Request, resp: GripperAngleMove.Response
    ):
        (success, msg) = self.manipulation_driver.open_gripper_to_angle(
            req.gripper_angle
        )
        resp.success = success
        resp.message = msg
        return resp

    def publish_hand_images(self, _):
        images = self.manipulation_driver.latest_hand_images
        if images is None:
            return

        for image in images:
            if image.source.name == "hand_image":
                pub_img = self._hand_image_pub
                pub_info = self._hand_image_info_pub
            elif image.source.name == "hand_depth":
                pub_img = self._hand_depth_map_pub
                pub_info = self._hand_depth_map_info_pub
            elif image.source.name == "hand_color_image":
                pub_img = self._hand_4k_image_pub
                pub_info = self._hand_4k_image_info_pub
            elif image.source.name == "hand_depth_in_hand_color_frame":
                pub_img = self._hand_4k_depth_map_pub
                pub_info = self._hand_4k_depth_map_info_pub

            if pub_img.get_subscription_count() > 0:
                img_msg, info_msg, _ = getImageMsg(
                    image, self.manipulation_driver.lease_manager
                )
                pub_img.publish(img_msg)
                pub_info.publish(info_msg)

    def publish_joint_states(self, _):
        state = self.manipulation_driver.kinematic_state
        joint_states = JointStatesToMsg(state, self.manipulation_driver.lease_manager)
        self._joint_state_pub.publish(joint_states)
