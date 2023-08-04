#! /usr/bin/env python
##############################################################################
#      Title     : follow_joint_trajectory_action_server.py
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

## 

##############################################################################
##                                                                          ##
## (Extended version of the follow_joint_trajectory_action_server.py)       ##
##                                                                          ##
##                                                                          ##
##############################################################################

import logging
import sys
import threading
from importlib import reload
import actionlib
import rospy
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryFeedback,
                              FollowJointTrajectoryResult)
from geometry_msgs.msg import Twist, TwistStamped, Pose, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse
from spot_msgs.srv import Dock, DockResponse, GetDockState, GetDockStateResponse

from spot_manipulation_driver.manipulation_driver_util import \
    SpotManipulationDriver
from bosdyn.client import math_helpers
import tf2_ros

class NRGSpotDriver(SpotManipulationDriver):
    def __init__(self, argv):
        # Authenticate robot, claim lease, and power on
        SpotManipulationDriver.authenticate_robot(self, argv)
        SpotManipulationDriver.init_clients(self)
        SpotManipulationDriver.claim(self)

        SpotManipulationDriver.verify_power_and_estop(self)
        # SpotManipulationDriver.stand_robot(self)

        # Initialize action servers, joint state publisher, and ee velocity subscribers
        self.arm_action_server = actionlib.SimpleActionServer(
            "spot/arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            self.arm_goal_callback,
            False,
        )
        self.finger_action_server = actionlib.SimpleActionServer(
            "spot/finger_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            self.finger_goal_callback,
            False,
        )

        self.joint_states_pub = rospy.Publisher(
            "joint_states", JointState, queue_size=10
        )

        self.ee_vel_sub = rospy.Subscriber(
            "/filtered_twist", Twist, self.ee_vel_sub_callback, queue_size=1
        )

        self.ap_ee_vel_sub = rospy.Subscriber(
            "ee_twist_cmds", TwistStamped, self.ap_ee_vel_sub_callback, queue_size=1
        )

        self.cmd_vel_sub = rospy.Subscriber(
            "cmd_vel", Twist, self.cmd_vel_callback, queue_size=1
        )

        self.go_to_pose_sub = rospy.Subscriber(
            "go_to_pose", PoseStamped, self.trajectory_callback, queue_size=1
        )

        rospy.Service("sit", Trigger, self.sit_callback)
        rospy.Service("stand", Trigger, self.stand_callback)

        # EStop
        rospy.Service("estop/hard", Trigger, self.estop_hard_callback)
        rospy.Service("estop/soft", Trigger, self.estop_soft_callback)
        rospy.Service("estop/release", Trigger, self.estop_release_callback)

        # Docking
        rospy.Service("dock", Dock, self.handle_dock)
        rospy.Service("undock", Trigger, self.handle_undock)
        # rospy.Service("docking_state", GetDockState, self.handle_get_docking_state)

        # Arm Services 
        rospy.Service("arm_stow", Trigger, self.arm_stow_callback)
        rospy.Service("arm_unstow", Trigger, self.arm_unstow_callback)
        rospy.Service("gripper_open", Trigger, self.gripper_open_callback)
        rospy.Service("gripper_close", Trigger, self.gripper_close_callback)
        # rospy.Service("gripper_angle_open", GripperAngleMove, self.gripper_angle_open_callback)
        
        # Start action servers
        self.arm_action_server.start()
        self.finger_action_server.start()

        # Action messages and helper attributes
        # Arm-related attributes
        self.arm_feedback = FollowJointTrajectoryFeedback()
        self.arm_result = FollowJointTrajectoryResult()
        self.arm_feedback_publish_flag = None

        # Finger-related attributes
        self.finger_feedback = FollowJointTrajectoryFeedback()
        self.finger_result = FollowJointTrajectoryResult()
        self.finger_feedback_publish_flag = None

        self.rate = rospy.Rate(10)

        # Hand image
        self.hand_image_mono_pub = rospy.Publisher(
            "camera/hand_mono/image", Image, queue_size=10
        )
        self.hand_image_color_pub = rospy.Publisher(
            "camera/hand_color/image", Image, queue_size=10
        )
        # Camera Info
        self.hand_image_mono_info_pub = rospy.Publisher(
            "camera/hand_mono/camera_info", CameraInfo, queue_size=10
        )
        self.hand_image_color_info_pub = rospy.Publisher(
            "camera/hand_color/camera_info", CameraInfo, queue_size=10
        )

    def arm_goal_callback(self, goal):
        """Callback for arm_action_server"""

        success = True

        # Translate message, and execute trajectory while publishing feedback
        traj_point_positions, traj_point_velocities, time_since_ref = SpotManipulationDriver.convert_ros_trajectory_msg(
            self, goal
        )

        self.arm_feedback_publish_flag = True
        arm_feedback_thread = threading.Thread(
            target=self.arm_follow_joint_trajectory_feedback
        )
        arm_feedback_thread.start()

        SpotManipulationDriver.arm_long_trajectory_executor(
            self, traj_point_positions, traj_point_velocities, time_since_ref
        )

        self.arm_feedback_publish_flag = False
        arm_feedback_thread.join()

        if success:
            rospy.loginfo("Successfully executed trajectory")
            self.arm_action_server.set_succeeded(self.arm_result)

    def finger_goal_callback(self, goal):
        """Callback for finger_action_server"""

        success = True

        # Translate message, and execute trajectory while publishing feedback
        traj_point_positions = []
        time_since_ref = []

        for i in range(0, len(goal.trajectory.points)):
            traj_point_positions.append(goal.trajectory.points[i].positions[0])
            time_since_ref.append(goal.trajectory.points[i].time_from_start.to_sec())

        self.finger_feedback_publish_flag = True
        finger_feedback_thread = threading.Thread(
            target=self.finger_follow_joint_trajectory_feedback
        )
        finger_feedback_thread.start()

        SpotManipulationDriver.gripper_trajectory_executor_with_time_control(
            self, traj_point_positions, time_since_ref
        )

        self.finger_feedback_publish_flag = False
        finger_feedback_thread.join()

        if success:
            rospy.loginfo("Successfully executed trajectory")
            self.finger_action_server.set_succeeded(self.finger_result)

    def arm_follow_joint_trajectory_feedback(self):
        """Feedback for arm_action_server"""
        # Publishes actual states of the joints

        while self.arm_feedback_publish_flag:
            # Get joint states
            joint_states_source = SpotManipulationDriver.get_joint_states(self)
            self.arm_feedback.header.stamp = rospy.Time(
                joint_states_source[0].seconds, joint_states_source[0].nanos
            )
            self.arm_feedback.joint_names = joint_states_source[1]
            self.arm_feedback.actual.positions = joint_states_source[2]
            self.arm_feedback.actual.velocities = joint_states_source[3]

            # Publish and sleep
            self.arm_action_server.publish_feedback(self.arm_feedback)
            self.rate.sleep()

    def finger_follow_joint_trajectory_feedback(self):
        """Feedback for arm_action_server"""
        # Publishes actual states of the joints

        while self.finger_feedback_publish_flag:
            # Get joint states
            joint_states_source = SpotManipulationDriver.get_joint_states(self)
            self.finger_feedback.header.stamp = rospy.Time(
                joint_states_source[0].seconds, joint_states_source[0].nanos
            )
            self.finger_feedback.joint_names = joint_states_source[1]
            self.finger_feedback.actual.positions = joint_states_source[2]
            self.finger_feedback.actual.velocities = joint_states_source[3]

            # Publish and sleep
            self.finger_action_server.publish_feedback(self.finger_feedback)
            self.rate.sleep()

    def publish_joint_states(self):
        """Method to constantly publish joint states"""
        joint_states = JointState()
        while not rospy.is_shutdown():
            # Get joint states
            joint_states_source = SpotManipulationDriver.get_joint_states(self)
            joint_states.header.stamp = rospy.Time(
                joint_states_source[0].seconds, joint_states_source[0].nanos
            )
            joint_states.name = joint_states_source[1]
            joint_states.position = joint_states_source[2]
            joint_states.velocity = joint_states_source[3]
            joint_states.effort = joint_states_source[4]

            # Publish and sleep
            self.joint_states_pub.publish(joint_states)
            self.rate.sleep()

    def ee_vel_sub_callback(self, msg):
        """Callback for end effector velocity command subscriber"""
        SpotManipulationDriver.ee_velocity_msg_executor(self, msg)

    def ap_ee_vel_sub_callback(self, msg):
        """Callback for Affordance Primitive end effector velocity command subscriber"""
        twist_msg = msg.twist        
        SpotManipulationDriver.ee_velocity_msg_executor(self, twist_msg)

    def cmd_vel_callback(self, msg):
        """Callback to send velocity commands to the spot base"""
        SpotManipulationDriver.walk_robot(self, msg)

    def handle_dock(self, req):
        """Dock the robot"""
        resp = SpotManipulationDriver.dock(self, req.dock_id)
        return DockResponse(resp[0], resp[1])

    def handle_undock(self, req):
        """Undock the robot"""
        resp = SpotManipulationDriver.undock(self)
        return TriggerResponse(resp[0], resp[1])

    def arm_stow_callback(self, req):
        response = SpotManipulationDriver.stow_arm(self)
        return TriggerResponse(response[0], response[1])

    def arm_unstow_callback(self, req):
        response = SpotManipulationDriver.unstow_arm(self)
        return TriggerResponse(response[0], response[1])

    def gripper_open_callback(self, req):
        response = SpotManipulationDriver.open_gripper(self)
        return TriggerResponse(response[0], response[1])

    def gripper_close_callback(self, req):
        response = SpotManipulationDriver.close_gripper(self)
        return TriggerResponse(response[0], response[1])

    # From the spot_ros.py driver

    def sit_callback(self, req):
        response = SpotManipulationDriver.sit_robot(self)
        return TriggerResponse(response[0], response[1])

    def stand_callback(self, req):
        response = SpotManipulationDriver.stand_robot(self)
        return TriggerResponse(response[0], response[1])

    def estop_hard_callback(self, req):
        """ROS service handler to hard-eStop the robot.  The robot will immediately cut power to the motors"""
        response = SpotManipulationDriver.estop_hard(self)
        return TriggerResponse(response[0], response[1])

    def estop_soft_callback(self, req):
        """ROS service handler to soft-eStop the robot.  The robot will try to settle on the ground before cutting
        power to the motors"""
        response = SpotManipulationDriver.estop_soft(self)
        return TriggerResponse(response[0], response[1])

    def estop_release_callback(self, req):
        """ROS service handler to disengage the eStop on the robot."""
        response = SpotManipulationDriver.disengage_estop(self)
        return TriggerResponse(response[0], response[1])


    def publish_hand_image(self):        
        image_msg1 = Image()
        while not rospy.is_shutdown():
            data = SpotManipulationDriver.get_hand_images(self)
            if data:                
            #     mage_msg0, camera_info_msg0 = getImageMsg(data[0], self.spot_wrapper)
            #     self.hand_image_mono_pub.publish(mage_msg0)
            #     self.hand_image_mono_info_pub.publish(camera_info_msg0)

            #     image_msg2, camera_info_msg2 = getImageMsg(data[1], self.spot_wrapper)
                image_msg1.header.frame_id = "Test"
                image_msg1.height = data[1].shot.image.rows
                image_msg1.width = data[1].shot.image.cols
                image_msg1.encoding = "rgb8"
                image_msg1.is_bigendian = True
                image_msg1.step = 3 * data[1].shot.image.cols
                image_msg1.data = data[1].shot.image.data
                self.hand_image_color_pub.publish(image_msg1)
            #     self.hand_image_color_info_pub.publish(camera_info_msg1)
            self.rate.sleep()

    # From spot_ros.py
    def trajectory_callback(self, msg):
        """
        Handle a callback from the trajectory topic requesting to go to a location
        The trajectory will time out after 5 seconds
        Args:
            msg: PoseStamped containing desired pose
        Returns:
        """
        try:
            self.send_trajectory_command(
                self.transform_pose_to_body_frame(msg), rospy.Duration(5)
            )
        except tf2_ros.LookupException as e:
            rospy.logerr(str(e))
    
    def send_trajectory_command(self, pose, duration, precise=True):
        """
        Send a trajectory command to the robot
        Args:
            pose: Pose the robot should go to. Must be in the body frame
            duration: After this duration, the command will time out and the robot will stop
            precise: If true, the robot will position itself precisely at the target pose, otherwise it will end up
                     near (within ~0.5m, rotation optional) the requested location
        Returns: (bool, str) tuple indicating whether the command was successfully sent, and a message
        """
        if pose.header.frame_id != "body":
            rospy.logerr("Trajectory command poses must be in the body frame")
            return

        return SpotManipulationDriver.trajectory_cmd(
            self,
            goal_x=pose.pose.position.x,
            goal_y=pose.pose.position.y,
            goal_heading=math_helpers.Quat(
                w=pose.pose.orientation.w,
                x=pose.pose.orientation.x,
                y=pose.pose.orientation.y,
                z=pose.pose.orientation.z,
            ).to_yaw(),
            cmd_duration=duration.to_sec()
        )

    def transform_pose_to_body_frame(self, pose):
        """
        Transform a pose to the body frame

        Args:
            pose: PoseStamped to transform

        Raises: tf2_ros.LookupException if the transform lookup fails

        Returns: Transformed pose in body frame if given pose is not in the body frame, or the original pose if it is
        in the body frame
        """
        if pose.header.frame_id == "body":
            return pose

        body_to_fixed = self.tf_buffer.lookup_transform(
            "body", pose.header.frame_id, rospy.Time()
        )

        pose_in_body = tf2_geometry_msgs.do_transform_pose(pose, body_to_fixed)
        pose_in_body.header.frame_id = "body"

        return pose_in_body

def main(argv):
    rospy.loginfo("node")
    nrg_spot_driver = NRGSpotDriver(argv)
    
    joint_states_pub_thread = threading.Thread(
        target=nrg_spot_driver.publish_joint_states
    )
    
    # hand_image_thread = threading.Thread(
    #     target=nrg_spot_driver.publish_hand_image
    # )

    # joint_states_pub_thread.setDaemon(True)
    # hand_image_thread.setDaemon(True)

    joint_states_pub_thread.start()
    # hand_image_thread.start()

    while not rospy.is_shutdown():
        rospy.spin()

    joint_states_pub_thread.join()
    # hand_image_thread.join()
    nrg_spot_driver.disconnect()


if __name__ == "__main__":
    rospy.init_node("nrg_spot_driver")
    reload(logging)
    argv = rospy.myargv(argv=sys.argv)
    if not main(argv[1:]):
        sys.exit(1)
