from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist, TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node


class MyNode(Node):

    def __init__(self):
        super().__init__('my_node')

        # Initialize your node here
        self.get_logger().info('My ROS 2 node is running.')

    def run(self):
        # Your main loop or logic goes here
        self.get_logger().info('Running...')
        rosT_o2f = TransformStamped()
        rosT_o2f.transform.rotation.x = 0.0
        rosT_o2f.transform.rotation.y = 0.0
        rosT_o2f.transform.rotation.z = 0.383
        rosT_o2f.transform.rotation.w = 0.924
        rosT_o2f.transform.translation.x = 1.0
        rosT_o2f.transform.translation.y = 2.0
        rosT_o2f.transform.translation.z = 3.0
        self.get_logger().info(f"Rotation x: {rosT_o2f.transform.rotation.x}")
        self.get_logger().info(f"Rotation y: {rosT_o2f.transform.rotation.y}")
        self.get_logger().info(f"Rotation z: {rosT_o2f.transform.rotation.z}")
        self.get_logger().info(f"Rotation w: {rosT_o2f.transform.rotation.w}")
        self.get_logger().info(f"Translation x: {rosT_o2f.transform.translation.x}")
        self.get_logger().info(f"Translation y: {rosT_o2f.transform.translation.y}")
        self.get_logger().info(f"Translation z: {rosT_o2f.transform.translation.z}")

        T_o2f, yaw_o2f = self.convert_transformstamped_to_matrix(rosT_o2f)
        self.get_logger().info(f"Transform as np matrix: \n{T_o2f}")
        self.get_logger().info(f"Yaw_o2f: \n{yaw_o2f}")


    def convert_transformstamped_to_matrix(self, transform_stamped):
        # Extract quaternion and translation components
        quat_x = transform_stamped.transform.rotation.x
        quat_y = transform_stamped.transform.rotation.y
        quat_z = transform_stamped.transform.rotation.z
        quat_w = transform_stamped.transform.rotation.w
        x = transform_stamped.transform.translation.x
        y = transform_stamped.transform.translation.y
        z = transform_stamped.transform.translation.z

        # Construct HTM representation
        rot = R.from_quat([quat_x, quat_y, quat_z, quat_w])
        euler_o2f = rot.as_euler('zxy')
        T_o2f = np.eye(4)
        self.get_logger().info(f"Identity inside func: \n {T_o2f}")
        T_o2f[:3, :3] = rot.as_matrix()  # Assign rotation matrix
        T_o2f[:3, 3] = [x, y, z]       # Assign translation vector

        return T_o2f, euler_o2f[0]



def main(args=None):
    rclpy.init(args=args)

    my_node = MyNode()

    try:
        my_node.run()
    finally:
        my_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
