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
import sys
import threading
import time

import rclpy
import rclpy.callback_groups
from rclpy.action.server import ServerGoalHandle
from builtin_interfaces.msg import Time as ROSTime
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist, TwistStamped, WrenchStamped
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_srvs.srv import Trigger

from spot_driver.spot_lease_manager import SpotLeaseManager
from spot_driver.spot_ros import CameraPubs
from spot_manipulation_driver.manipulation_driver import \
    SpotManipulationDriver
import spot_manipulation_driver.ros_helpers as ros_helpers
from spot_driver.ros_helpers import getImageMsg, JointStatesToMsg
from spot_msgs.msg import ManipulatorState
from spot_msgs.srv import GripperAngleMove


class FollowJointTrajectoryActionServer(Node):
    def __init__(self, argv):

        Node.__init__(self, "follow_joint_trajectory_node")

        self.manipulation_driver: SpotManipulationDriver = None

        # Declare ROS parameters
        self.declare_parameter('hostname', 'default_value',
            ParameterDescriptor(description='Spot computer hostname.',
                                type=ParameterType.PARAMETER_STRING,
                                read_only=True))
        
        self.declare_parameter('rates.status.arm_state', 1.0,
            ParameterDescriptor(description='Publish rate for arm status topic',
                                type=ParameterType.PARAMETER_DOUBLE,
                                floating_point_range=[FloatingPointRange(
                                    from_value=0.0, to_value=1000.0, step=0.0)],
                                read_only=True))
        
        self.declare_parameter('publish_joint_states', False,
            ParameterDescriptor(description='Whether or not to publish the joint states',
                                type=ParameterType.PARAMETER_BOOL,
                                read_only=True))
        
        self.declare_parameter('rates.status.joint_state', 1.0,
            ParameterDescriptor(description='Publish rate for joint state topic',
                                type=ParameterType.PARAMETER_DOUBLE,
                                floating_point_range=[FloatingPointRange(
                                    from_value=0.0, to_value=1000.0, step=0.0)],
                                read_only=True))

        # --- Initialize action messages --- #

        # Arm-related attributes
        self.arm_feedback = FollowJointTrajectory.Feedback()
        self.arm_result = FollowJointTrajectory.Result()
        self.arm_feedback_publish_flag = False

        # Finger-related attributes
        self.finger_feedback = FollowJointTrajectory.Feedback()
        self.finger_result = FollowJointTrajectory.Result()
        self.finger_feedback_publish_flag = False

    def connect(self, lease_manager: SpotLeaseManager) -> bool:
        self.get_logger().info("Connecting manipulation driver")
        self.manipulation_driver = SpotManipulationDriver(self.get_logger(), self.get_parameter('hostname'))
        
        publish_joint_states = self.get_parameter_or('publish_joint_states', False)

        self.get_logger().info("Setting arm state callbacks")
        callbacks = {
            'arm_state'  : self.ArmStateCB,
            'hand_image' : self.publishHandImages
        }
        rates = {
            'arm_state'  : self.get_parameter('rates.status.arm_state'),
            'hand_image' : self.get_parameter('rates.sensors.hand_image')   
        }
        if publish_joint_states:
            callbacks['publish_joint_state'] = self.publishJointStates
            rates['publish_joint_state'] = self.get_parameter_or('rates.status.joint_state', 1.0)

        if self.manipulation_driver.connect(lease_manager, rates, callbacks):
            self.get_logger().info(f"Connected to Spot {lease_manager.ID}")
        else:
            self.get_logger().fatal("Failed to launch Spot manipulation driver")
            return False

        # Create a control group to prevent multiple callbacks from commanding motion simultaneously
        motion_callback_group  = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        gripper_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup() 

        # Create data publishers and subscribers
        self._hand_image_4k_pub              = self.create_publisher(Image           , "~/images/4k_gray"             , 10)
        self._hand_image_4k_rgb_pub          = self.create_publisher(Image           , "~/images/4k_rgb"              , 10)
        self._hand_image_depth_pub           = self.create_publisher(Image           , "~/images/depth"               , 10)
        self._hand_image_depth_gray_pub      = self.create_publisher(Image           , "~/images/depth_grayscale"     , 10)
        self._hand_image_4k_info_pub         = self.create_publisher(CameraInfo      , "~/camera_info/4k_rgb"         , 10)
        self._hand_image_4k_rgb_info_pub     = self.create_publisher(CameraInfo      , "~/camera_info/4k_gray"        , 10)
        self._hand_image_depth_info_pub      = self.create_publisher(CameraInfo      , "~/camera_info/depth"          , 10)
        self._hand_image_depth_gray_info_pub = self.create_publisher(CameraInfo      , "~/camera_info/depth_grayscale", 10)
        self._force_torque_state_pub         = self.create_publisher(ManipulatorState, "~/manipulator_state"          , 10)

        if publish_joint_states:
            self._joint_state_pub = self.create_publisher(JointState, "~/joint_state", 10)

        self.ee_vel_sub    = self.create_subscription(Twist, "~/cmd_vel", self.ee_vel_sub_callback, 10, callback_group=motion_callback_group)
        self.ap_ee_vel_sub = self.create_subscription(TwistStamped, "/ee_twist_cmds", self.ap_ee_vel_sub_callback, 10,callback_group=motion_callback_group)

        # Create services for arm motions
        self.create_service(Trigger, "~/claim"        , self.claimCB)
        self.create_service(Trigger, "~/power_on"     , self.powerOnCB)
        self.create_service(Trigger, "~/unstow_arm"   , self.unstowServiceCB, callback_group=motion_callback_group)
        self.create_service(Trigger, "~/stow_arm"     , self.stowServiceCB, callback_group=motion_callback_group)
        self.create_service(Trigger, "~/close_gripper", self.gripperCloseServiceCB, callback_group=gripper_callback_group)
        self.create_service(Trigger, "~/open_gripper" , self.gripperOpenServiceCB, callback_group=gripper_callback_group)
        self.create_service(GripperAngleMove, "~/set_gripper_angle", self.gripperAngleServiceCB, callback_group=gripper_callback_group)

        # Initialize action servers
        self.arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            self.arm_goal_callback,
            callback_group=motion_callback_group
        )

        self.finger_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/finger_controller/follow_joint_trajectory",
            self.finger_goal_callback,
            callback_group=gripper_callback_group
        )

        return True

    def arm_goal_callback(self, goal_handle: ServerGoalHandle):
        """Callback for the /spot_arm/arm_controller/follow_joint_trajectory action server """

        self.get_logger().info("Executing goal for the /spot_arm/arm_controller/follow_joint_trajectory action server")

        success = True

        # Translate message and execute trajectory while publishing feedback
        traj_point_positions, traj_point_velocities, timepoints = ros_helpers.JointTrajectoryToLists(goal_handle.request)

        self.arm_feedback_publish_flag = True
        arm_feedback_thread = threading.Thread(
            target=self.provideJointTrajectoryFeedback, args=(goal_handle,)
        )
        arm_feedback_thread.start()

        self.manipulation_driver.executeLongHorizonTrajectory(
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

    def provideJointTrajectoryFeedback(self, goal_handle: ServerGoalHandle):
        """Feedback for arm action server"""
        rate = self.create_rate(frequency=2.0, clock=self.get_clock())

        # Publishes actual states of the joints
        while self.arm_feedback_publish_flag:
            self.arm_feedback = ros_helpers.getJointStateFeedback(self.manipulation_driver)
            goal_handle.publish_feedback(self.arm_feedback)
            # TODO: Remove logging statements once we know this works
            self.get_logger().info("Joint feedback thread sleeping")
            rate.sleep()
            self.get_logger().info("Joint feedback thread done sleeping")

    def finger_follow_joint_trajectory_feedback(self, goal_handle: ServerGoalHandle):
        """Feedback for finger action server"""
        # Publishes actual states of the joints

        while self.finger_feedback_publish_flag:
            # Get joint states
            self.finger_feedback = ros_helpers.getJointStateFeedback(self.manipulation_driver)

            # Publish and sleep
            goal_handle.publish_feedback(self.finger_feedback)
            time.sleep(0.5)  # TODO: Replace this sleep with ros2 compatible sleep

    def ee_vel_sub_callback(self, msg: Twist):
        """Callback for end effector velocity command subscriber"""
        self.manipulation_driver.ee_velocity_msg_executor(self, msg)

    def ap_ee_vel_sub_callback(self, msg: TwistStamped):
        """Callback for Affordance Primitive end effector velocity command subscriber"""
        self.ee_vel_sub_callback(msg.twist)

    def ArmStateCB(self):
        state = self.manipulation_driver.arm_state
        state_msg = ros_helpers.ManipulatorStatesToMsg(state, self.manipulation_driver)
        self._force_torque_state_pub.publish(state_msg)

    def claimCB(self, _: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.claim()
        resp.success = success
        resp.message = msg
        return resp
    
    def powerOnCB(self, _: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.verify_power_and_estop()
        resp.success = success
        resp.message = msg
        return resp

    def stowServiceCB(self, _: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.stow_arm()
        resp.success = success
        resp.message = msg
        return resp
    
    def unstowServiceCB(self, _: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.unstow_arm()
        resp.success = success
        resp.message = msg
        return resp
    
    def gripperCloseServiceCB(self, _, resp: Trigger.Response) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.close_gripper()
        resp.success = success
        resp.message = msg
        return resp

    def gripperOpenServiceCB(self, _, resp: Trigger.Response) -> Trigger.Response:
        (success, msg) = self.manipulation_driver.open_gripper()
        resp.success = success
        resp.message = msg
        return resp
    
    def gripperAngleServiceCB(self, req: GripperAngleMove.Request, resp: GripperAngleMove.Response):
        (success, msg) = self.manipulation_driver.open_gripper_to_angle(req.gripper_angle)
        resp.success = success
        resp.message = msg
        return resp
    
    def publishHandImages(self):
        for image in self.manipulation_driver.latest_hand_images:
            if image.source_name == "hand_image":
                pub_img  = self._hand_image_4k_pub
                pub_info = self._hand_image_4k_info_pub
            elif image.source.name == "hand_depth":
                pub_img  = self._hand_image_depth_pub
                pub_info = self._hand_image_depth_info_pub
            elif image.source.name == "hand_color_image":
                pub_img  = self._hand_image_4k_rgb_pub
                pub_info = self._hand_image_4k_rgb_info_pub
            elif image.source.name == "hand_depth_in_hand_color_frame":
                pub_img  = self._hand_image_depth_gray_pub
                pub_info = self._hand_image_depth_gray_info_pub

            if pub_img.get_subscription_count() > 0:
                img_msg, info_msg, _ = getImageMsg(image, self.manipulation_driver.lease_manager)
                pub_img.publish(img_msg)
                pub_info.publish(info_msg)

    def publishJointStates(self):
        state = self.manipulation_driver.robot_state
        joint_states = JointStatesToMsg(state.KinematicState, self.manipulation_driver.lease_manager)
        self._joint_state_pub.publish(joint_states)



# class JointStatePublisher(Node):
#     """Node to publish joint states"""

#     def __init__(self, follow_joint_trajectory_action_server):
#         super().__init__("joint_state_publisher_node")

#         # Initialize the action server node object and joint states publisher
#         self.follow_joint_trajectory_action_server = (
#             follow_joint_trajectory_action_server
#         )
#         self.joint_states_pub = self.create_publisher(
#             # JointState, "/spot_arm/joint_states", 10
#             JointState,
#             "/joint_states",
#             10,
#         )
#         self.force_torque_state_pub = self.create_publisher(
#             WrenchStamped,
#             "/ee_force",
#             10,
#         )
#         timer_period = 0.25
#         self.timer = self.create_timer(timer_period, self.publish_joint_states)

#     def publish_joint_states(self):
#         """Method to constantly publish joint states"""
#         joint_states = JointState()
#         force_torque_state = WrenchStamped()

#         # Get joint states and force-torque data
#         joint_states_source = (
#             self.follow_joint_trajectory_action_server.get_joint_states()
#         )

#         force_torque_state_source = self.follow_joint_trajectory_action_server.get_force_torque_state()
#         joint_states.header.stamp = ROSTime(
#             sec=joint_states_source[0].seconds, nanosec=joint_states_source[0].nanos
#         )
#         joint_states.name = joint_states_source[1]
#         joint_states.position = joint_states_source[2]
#         joint_states.velocity = joint_states_source[3]
#         joint_states.effort = joint_states_source[4]

#         force_torque_state.force.x = force_torque_state_source[0]
#         force_torque_state.force.y = force_torque_state_source[1]
#         force_torque_state.force.z = force_torque_state_source[2]

#         # Publish joint states and force-torque data
#         self.joint_states_pub.publish(joint_states)
#         self.force_torque_state_pub.publish(force_torque_state)



# def main():

#     argv = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]

#     rclpy.init(args=argv)
#     try:
#         follow_joint_trajectory_action_server = FollowJointTrajectoryActionServer(argv)
#         # joint_states_publisher = JointStatePublisher(
#         #     follow_joint_trajectory_action_server
#         # )
#         executor = MultiThreadedExecutor(num_threads=4)
#         executor.add_node(follow_joint_trajectory_action_server)
#         # executor.add_node(joint_states_publisher)

#         try:
#             # Spin the two nodes in separate threads
#             executor.spin()
#         finally:
#             executor.shutdown()
#             follow_joint_trajectory_action_server.disconnect()
#             follow_joint_trajectory_action_server.destroy_node()
#             # joint_states_publisher.destroy_node()
#     finally:
#         rclpy.shutdown()


# if __name__ == "__main__":
#     if not main():
#         sys.exit(1)
