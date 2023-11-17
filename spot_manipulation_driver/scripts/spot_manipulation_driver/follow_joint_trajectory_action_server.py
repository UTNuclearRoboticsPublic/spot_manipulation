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
import logging
import sys
import threading
from importlib import reload

import actionlib
import rospy
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryFeedback,
                              FollowJointTrajectoryResult)
from geometry_msgs.msg import Twist, TwistStamped
from image2grasp_msg.msg import (Image2GraspAction, Image2GraspFeedback,
                                 Image2GraspResult)
from sensor_msgs.msg import JointState
from spot_manipulation_driver.manipulation_driver_util import \
    SpotManipulationDriver


class FollowJointTrajectory(SpotManipulationDriver):
    def __init__(self, argv):
        # Authenticate robot, claim lease, and power on
        SpotManipulationDriver.authenticate_robot(self, argv)
        SpotManipulationDriver.init_clients(self)
        SpotManipulationDriver.claim(self)
        SpotManipulationDriver.verify_power_and_estop(self)
        SpotManipulationDriver.stand_robot(self)

        # Initialize action servers, joint state publisher, and ee velocity subscribers
        self.arm_action_server = actionlib.SimpleActionServer(
            "spot_arm/arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            self.arm_goal_callback,
            False,
        )
        self.finger_action_server = actionlib.SimpleActionServer(
            "spot_arm/finger_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            self.finger_goal_callback,
            False,
        )

        self.image_to_grasp_action_server = actionlib.SimpleActionServer(
            "spot/image_to_grasp",
            Image2GraspAction,
            self.image_to_grasp_goal_callback,
            False,
        )
        self.joint_states_pub = rospy.Publisher(
            "joint_states", JointState, queue_size=10
        )

        self.ee_vel_sub = rospy.Subscriber(
            "/spacenav/twist", Twist, self.ee_vel_sub_callback
        )

        self.ap_ee_vel_sub = rospy.Subscriber(
            "/ee_twist_cmds", TwistStamped, self.ap_ee_vel_sub_callback
        )

        # Start action servers
        self.arm_action_server.start()
        self.finger_action_server.start()
        self.image_to_grasp_action_server.start()

        # Action messages and helper attributes
        # Arm-related attributes
        self.arm_feedback = FollowJointTrajectoryFeedback()
        self.arm_result = FollowJointTrajectoryResult()
        self.arm_feedback_publish_flag = None

        # Finger-related attributes
        self.finger_feedback = FollowJointTrajectoryFeedback()
        self.finger_result = FollowJointTrajectoryResult()
        self.finger_feedback_publish_flag = None

        # image_to_grasp-related attributes
        self.image_to_grasp_feedback = Image2GraspFeedback()
        self.image_to_grasp_result = Image2GraspResult()

        self.rate = rospy.Rate(10)

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

    def image_to_grasp_goal_callback(self, goal):
        """Callback for image_to_grasp_action_server"""
        self.image_to_grasp_result = SpotManipulationDriver.image_to_grasp(
            self, goal.pixel_x, goal.pixel_y
        )

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


def main(argv):

    follow_joint_trajectory = FollowJointTrajectory(argv)
    joint_states_pub_thread = threading.Thread(
        target=follow_joint_trajectory.publish_joint_states()
    )
    joint_states_pub_thread.start()

    while not rospy.is_shutdown():
        rospy.spin()

    joint_states_pub_thread.join()
    follow_joint_trajectory.disconnect()


if __name__ == "__main__":
    rospy.init_node("follow_joint_trajectory_node")
    reload(logging)
    argv = rospy.myargv(argv=sys.argv)
    if not main(argv[1:]):
        sys.exit(1)
