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
from builtin_interfaces.msg import Time as ROSTime
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist, TwistStamped, Wrench, WrenchStamped
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from spot_manipulation_driver.manipulation_driver_util import \
    SpotManipulationDriver


class FollowJointTrajectoryActionServer(Node, SpotManipulationDriver):
    def __init__(self, argv):

        Node.__init__(self, "follow_joint_trajectory_node")

        # Authenticate robot, claim lease, and power on
        SpotManipulationDriver.authenticate_robot(self, argv)
        SpotManipulationDriver.init_clients(self)
        SpotManipulationDriver.forceClaim(self)
        SpotManipulationDriver.verify_power_and_estop(self)
        SpotManipulationDriver.stand_robot(self)

        # Initialize action servers and ee velocity subscribers
        self.arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            # "/spot_arm/arm_controller/follow_joint_trajectory",
            "/arm_controller/follow_joint_trajectory",
            self.arm_goal_callback,
        )
        self.finger_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            # "/spot_arm/finger_controller/follow_joint_trajectory",
            "/finger_controller/follow_joint_trajectory",
            self.finger_goal_callback,
        )
        self.ee_vel_sub = self.create_subscription(
            Twist, "/spacenav/twist", self.ee_vel_sub_callback, 10
        )
        self.ap_ee_vel_sub = self.create_subscription(
            TwistStamped, "/ee_twist_cmds", self.ap_ee_vel_sub_callback, 10
        )

        # Initialize action messages
        # Arm-related attributes
        self.arm_feedback = FollowJointTrajectory.Feedback()
        self.arm_result = FollowJointTrajectory.Result()
        self.arm_feedback_publish_flag = False

        # Finger-related attributes
        self.finger_feedback = FollowJointTrajectory.Feedback()
        self.finger_result = FollowJointTrajectory.Result()
        self.finger_feedback_publish_flag = False

    def arm_goal_callback(self, goal_handle):
        """Callback for the /spot_arm/arm_controller/follow_joint_trajectory action server """

        self.get_logger().info(
            "Executing goal for the /spot_arm/arm_controller/follow_joint_trajectory action server"
        )

        success = True

        # Translate message and execute trajectory while publishing feedback
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

    def arm_follow_joint_trajectory_feedback(self, goal_handle):
        """Feedback for arm action server"""
        # Publishes actual states of the joints

        while self.arm_feedback_publish_flag:
            # Get joint states
            joint_states_source = SpotManipulationDriver.get_joint_states(self)
            self.finger_feedback.header.stamp = ROSTime(
                sec=joint_states_source[0].seconds, nanosec=joint_states_source[0].nanos
            )
            self.arm_feedback.joint_names = joint_states_source[1]
            self.arm_feedback.actual.positions = joint_states_source[2]
            self.arm_feedback.actual.velocities = joint_states_source[3]

            # Publish and sleep
            goal_handle.publish_feedback(self.arm_feedback)
            time.sleep(0.5)  # TODO: Replace this sleep with ros2 compatible sleep

    def finger_follow_joint_trajectory_feedback(self, goal_handle):
        """Feedback for finger action server"""
        # Publishes actual states of the joints

        while self.finger_feedback_publish_flag:
            # Get joint states
            joint_states_source = SpotManipulationDriver.get_joint_states(self)
            self.finger_feedback.header.stamp = ROSTime(
                sec=joint_states_source[0].seconds, nanosec=joint_states_source[0].nanos
            )
            self.finger_feedback.joint_names = joint_states_source[1]
            self.finger_feedback.actual.positions = joint_states_source[2]
            self.finger_feedback.actual.velocities = joint_states_source[3]

            # Publish and sleep
            goal_handle.publish_feedback(self.finger_feedback)
            time.sleep(0.5)  # TODO: Replace this sleep with ros2 compatible sleep

    def ee_vel_sub_callback(self, msg):
        """Callback for end effector velocity command subscriber"""
        SpotManipulationDriver.ee_velocity_msg_executor(self, msg)

    def ap_ee_vel_sub_callback(self, msg):
        """Callback for Affordance Primitive end effector velocity command subscriber"""
        twist_msg = msg.twist
        SpotManipulationDriver.ee_velocity_msg_executor(self, twist_msg)


class JointStatePublisher(Node):
    """Node to publish joint states"""

    def __init__(self, follow_joint_trajectory_action_server):
        super().__init__("joint_state_publisher_node")

        # Initialize the action server node object and joint states publisher
        self.follow_joint_trajectory_action_server = (
            follow_joint_trajectory_action_server
        )
        self.joint_states_pub = self.create_publisher(
            # JointState, "/spot_arm/joint_states", 10
            JointState,
            "/joint_states",
            10,
        )
        self.force_torque_state_pub = self.create_publisher(
            WrenchStamped,
            "/ee_force",
            10,
        )
        timer_period = 0.25
        self.timer = self.create_timer(timer_period, self.publish_joint_states)

    def publish_joint_states(self):
        """Method to constantly publish joint states"""
        joint_states = JointState()
        force_torque_state = WrenchStamped()

        # Get joint states and force-torque data
        joint_states_source = (
            self.follow_joint_trajectory_action_server.get_joint_states()
        )
        force_torque_state_source = self.follow_joint_trajectory_action_server.get_force_torque_state()
        
        joint_states.header.stamp = ROSTime(
            sec=joint_states_source[0].seconds, nanosec=joint_states_source[0].nanos
        )
        joint_states.name = joint_states_source[1]
        joint_states.position = joint_states_source[2]
        joint_states.velocity = joint_states_source[3]
        joint_states.effort = joint_states_source[4]

        force_torque_state.header.stamp = ROSTime(
            sec=joint_states_source[0].seconds, nanosec=joint_states_source[0].nanos
        )
        force_torque_state.wrench.force.x = force_torque_state_source[0]
        force_torque_state.wrench.force.y = force_torque_state_source[1]
        force_torque_state.wrench.force.z = force_torque_state_source[2]

        # Publish joint states and force-torque data
        self.joint_states_pub.publish(joint_states)
        self.force_torque_state_pub.publish(force_torque_state)



def main():

    argv = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]

    rclpy.init(args=argv)
    try:
        follow_joint_trajectory_action_server = FollowJointTrajectoryActionServer(argv)
        joint_states_publisher = JointStatePublisher(
            follow_joint_trajectory_action_server
        )
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(follow_joint_trajectory_action_server)
        executor.add_node(joint_states_publisher)

        try:
            # Spin the two nodes in separate threads
            executor.spin()
        finally:
            executor.shutdown()
            follow_joint_trajectory_action_server.disconnect()
            follow_joint_trajectory_action_server.destroy_node()
            joint_states_publisher.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    if not main():
        sys.exit(1)
