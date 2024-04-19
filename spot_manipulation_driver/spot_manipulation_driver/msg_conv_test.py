from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist, TransformStamped
import numpy as np
import rclpy
from rclpy.time import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


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
        rosT_o2f.transform.rotation.z = 0.7068252
        rosT_o2f.transform.rotation.w = 0.7073883
        rosT_o2f.transform.translation.x = 1.0
        rosT_o2f.transform.translation.y = 1.0
        rosT_o2f.transform.translation.z = 1.0
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

        x = 1.0
        y = 2.0
        yaw_f2b = 0.0
        p_o = self.transform_planar_point_and_or(x, y, yaw_f2b, T_o2f, yaw_o2f)
        self.get_logger().info(f"p_o: \n{p_o}")

        # Initialize a JointTrajectory message
        test_traj = JointTrajectory()
        test_traj.joint_names = ['arm0_elbow_pitch', 'arm0_elbow_roll', 'arm0_shoulder_pitch', 'arm0_shoulder_yaw', 'arm0_wrist_pitch', 'arm0_wrist_roll', 'body_or', 'body_x', 'body_y']
        point1 = JointTrajectoryPoint()
        point1.positions = [2.928600788116455, 0.011042356491088867, -2.635392189025879, -0.030835390090942383, -0.2719740867614746, -0.011610031127929688, 0.0, 0.0, 0.0]
        point1.time_from_start = Duration(seconds=0).to_msg()
        test_traj.points.append(point1)
        point2 = JointTrajectoryPoint()
        point2.positions = [2.923600788116455, 0.011023557561419654, -2.630892600251698, -0.03078275328019055, -0.27150959780991846, -0.01159018625052494, 1.6944611413756512e-07, 3.1046245015458756e-08, -1.5893811556126103e-07]
        point2.time_from_start = Duration(seconds=0.1).to_msg()
        test_traj.points.append(point2)
        self.get_logger().info("Here is the trajectory before conversion:")
        self.get_logger().info(f"Joint Names: {test_traj.joint_names}")
        self.get_logger().info(f"Joint positions: {test_traj.points}")
        traj_point_positions, traj_point_velocities, timepoints = self.wbc_joint_trajectory_to_lists(test_traj, T_o2f, yaw_o2f)
        self.get_logger().info(f"Here is the trajectory after conversion: \n{traj_point_positions}")

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
        print("Euler zxy conv: ", euler_o2f)
        T_o2f = np.eye(4)
        T_o2f[:3, :3] = rot.as_matrix()  # Assign rotation matrix
        T_o2f[:3, 3] = [x, y, z]       # Assign translation vector

        return T_o2f, euler_o2f[0]


    def transform_planar_point_and_or(self, x, y, yaw_f2b, T_o2f, yaw_o2f):


        # Create HTM representation base_footprint to body transformation
        T_f2b = np.eye(4)

        # Assign rotation to the HTM
        R_f2b = R.from_euler('z', yaw_f2b)
        T_f2b[:3, :3] = R_f2b.as_matrix()

        # Assign translation to the HTM
        p_b = [x, y, 0]  # z 0 since footprint
        T_f2b[:3, 3] = p_b

        # Compute HTM representing body in odom
        T_o2b = T_o2f @ T_f2b

        p_o = [T_o2b[0, 3], T_o2b[1, 3], yaw_o2f + yaw_f2b]  # Extract transformed x, y, and updated yaw

        return p_o

    def wbc_joint_trajectory_to_lists(self, msg: JointTrajectory, T_o2f, yaw_o2f):
        traj_point_positions = []
        traj_point_velocities = []
        timepoints = []

        # Order of joints
        joint_order = [
            "body_x",
            "body_y",
            "body_or",
            "arm0_shoulder_yaw",
            "arm0_shoulder_pitch",
            "arm0_elbow_pitch",
            "arm0_elbow_roll",
            "arm0_wrist_pitch",
            "arm0_wrist_roll",
        ]

        # Reorder joint commands based on joint_order and put them into long lists of lists
        point: JointTrajectoryPoint
        for point in msg.points:
            pos_dict = {}
            # vel_dict = {}

            for j in range(0, len(joint_order)):
                name = msg.joint_names[j]
                print("Joint name: ", name)
                pos_dict[name] = point.positions[j]
                # vel_dict[name] = point.velocities[j]

            traj_point_positions.append(
                list(map(lambda joint_name: pos_dict[joint_name], joint_order))
            )

            # Transform the body joints from body to odom frame
            [x,y,theta] = traj_point_positions[-1][:3]
            traj_point_positions[-1][:3] = self.transform_planar_point_and_or(x, y, theta, T_o2f, yaw_o2f)

            # traj_point_velocities.append(
            #     list(map(lambda joint_name: vel_dict[joint_name], joint_order))
            # )
            timepoints.append(
                point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            )

        return traj_point_positions, traj_point_velocities, timepoints



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
