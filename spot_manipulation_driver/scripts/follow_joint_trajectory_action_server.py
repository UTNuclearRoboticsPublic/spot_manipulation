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

import rclpy
from action_tutorials_interfaces.action import Fibonacci
# from control_msgs.msgs import (FollowJointTrajectoryAction,
#                               FollowJointTrajectoryFeedback,
#                               FollowJointTrajectoryResult)
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import JointState

from spot_manipulation_driver.manipulation_driver_util import \
    SpotManipulationDriver


class FollowJointTrajectory(Node, SpotManipulationDriver):
    # Initialize action servers and joint state publisher, and start action servers
    # Arm-related attributes
    arm_feedback = FollowJointTrajectory.Feedback()
    arm_result = FollowJointTrajectory.Result()
    arm_feedback_publish_flag = None

    # Finger-related attributes
    finger_feedback = FollowJointTrajectory.Feedback()
    finger_result = FollowJointTrajectory.Result()
    finger_feedback_publish_flag = None

    # Joint state publisher
    joint_states_pub = None

    # Other helper attributes
    rate = None

    def __init__(self, argv):
        Node.__init__(self, "follow_joint_trajectory_node")
        # Authenticate robot, claim lease, and power on
        # SpotManipulationDriver.authenticate_robot(self, argv)
        # SpotManipulationDriver.init_clients(self)
        # SpotManipulationDriver.claim(self)
        # SpotManipulationDriver.verify_power_and_estop(self)

        print("Before initializing arm action server")
        print(type(ActionServer))
        # self.arm_action_server = ActionServer(
        #     self,
        #     Fibonacci,
        #     "/spot_arm/arm_controller/follow_joint_trajectory",
        #     self.arm_goal_callback,
        # )
        self.arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/spot_arm/arm_controller/follow_joint_trajectory",
            self.arm_goal_callback,
        )
        print("After initializing arm action server")
        self.finger_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/spot_arm/finger_controller/follow_joint_trajectory",
            self.arm_goal_callback,
        )

        self.joint_states_pub = self.create_publisher(
            JointState, "/spot_arm/joint_states", queue_size=10
        )

        self.arm_action_server.start()
        self.finger_action_server.start()

        self.rate = rospy.Rate(10)

    def arm_goal_callback(self, goal_handle):
        """Callback for arm_action_server"""

        self.get_logger().info("Executing follow_joint_trajectory arm goal")

        success = True

        # Translate message, and execute trajectory while publishing feedback
        # TODO: Implement goal preemption
        traj_point_positions, traj_point_velocities, time_since_ref = SpotManipulationDriver.convert_ros_trajectory_msg(
            self, goal_handle.request
        )

        self.arm_feedback_publish_flag = True
        arm_feedback_thread = threading.Thread(
            target=self.arm_follow_joint_trajectory_feedback, args=(goal_handle,)
        )
        arm_feedback_thread.start()

        SpotManipulationDriver.arm_long_trajectory_executor(
            self, traj_point_positions, traj_point_velocities, time_since_ref
        )

        self.arm_feedback_publish_flag = False
        arm_feedback_thread.join()

        if success:
            goal_handle.succeed()
            self.get_logger().info("Successfully executed trajectory")
        return self.arm_result

    def finger_goal_callback(self, goal_handle):
        """Callback for finger_action_server"""

        self.get_logger().info("Executing follow_joint_trajectory finger goal")

        success = True

        # Translate message, and execute trajectory while publishing feedback
        # TODO: Implement goal preemption
        traj_point_positions = []
        time_since_ref = []

        for i in range(0, len(goal_handle.request.trajectory.points)):
            traj_point_positions.append(
                goal_handle.request.trajectory.points[i].positions[0]
            )
            time_since_ref.append(
                goal_handle.request.trajectory.points[i].time_from_start.to_sec()
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
            rclpy.get_logger().info("Successfully executed trajectory")
        return self.finger_result

    def arm_follow_joint_trajectory_feedback(self, goal_handle):
        """Feedback for arm_action_server"""
        # Currently feedback only publishes actual states of the joints. TODO: Publish desired states and errors

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
            goal_handle.publish_feedback(self.arm_feedback)
            self.rate.sleep()

    def finger_follow_joint_trajectory_feedback(self):
        """Feedback for arm_action_server"""
        # Currently feedback only publishes actual states of the joints. TODO: Publish desired states and errors

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
            goal_handle.publish_feedback(self.finger_feedback)
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


# def main(argv):
def main():

    argv = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]

    rclpy.init(args=argv)
    follow_joint_trajectory = FollowJointTrajectory(argv)
    joint_states_pub_thread = threading.Thread(
        target=follow_joint_trajectory.publish_joint_states()
    )
    joint_states_pub_thread.start()

    rclpy.spin(follow_joint_trajectory)
    rclpy.shutdown()

    joint_states_pub_thread.join()
    follow_joint_trajectory.disconnect()


if __name__ == "__main__":
    main()
    # # rospy.init_node("follow_joint_trajectory_node")
    # argv = rclpy.utilities.remove_ros_args(args=sys.argv)
    # if not main(argv[1:]):
    #     sys.exit(1)
