#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from spot_driver.spot_lease_manager import SpotLeaseManager
from spot_driver.spot_ros import SpotROS
from .spot_manipulation_driver_ros import SpotManipulationDriverROS

def main():
    rclpy.init()

    lease_manager = SpotLeaseManager()
    arm_node = SpotManipulationDriverROS()
    body_node = SpotROS()

    if body_node.connect(lease_manager):
        body_node.get_logger().info("Spot body ROS node connected")
        if arm_node.connect(lease_manager):
            arm_node.get_logger().info("Spot arm ROS node connected")

            # Spin up two threads for each node
            exec = MultiThreadedExecutor(num_threads=4)
            exec.add_node(body_node)
            exec.add_node(arm_node)
            exec.spin()

    arm_node.destroy_node()
    body_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()