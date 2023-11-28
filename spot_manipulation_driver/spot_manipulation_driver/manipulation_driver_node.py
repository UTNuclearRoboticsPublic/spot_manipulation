#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from spot_driver.spot_lease_manager import SpotLeaseManager
from .spot_manipulation_driver_ros import SpotManipulationDriverROS

def main():
    rclpy.init()

    lease_manager = SpotLeaseManager()
    arm_node = SpotManipulationDriverROS()

    if not arm_node.connect(lease_manager):
        arm_node.get_logger().fatal("Connection failed, shutting down node")
        return
    
    exec = MultiThreadedExecutor(num_threads=2)
    exec.add_node(arm_node)
    exec.spin()

    arm_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()