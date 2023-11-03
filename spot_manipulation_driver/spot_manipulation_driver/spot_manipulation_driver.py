#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from spot_driver.spot_lease_manager import SpotLeaseManager
from .spot_arm_node import SpotArmNode

def main():
    rclpy.init()

    lease_manager = SpotLeaseManager()
    arm_node = SpotArmNode()

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