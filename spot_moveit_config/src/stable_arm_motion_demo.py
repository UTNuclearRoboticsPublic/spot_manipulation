import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint

def main():
    node = Node('stable_arm_motion_demo')

    action_client = ActionClient(node, MoveGroup, '/spot_moveit/stable_move_action')
    if not action_client.wait_for_server(5.0):
        node.get_logger().error('Could not contact action server within 5 seconds')
        exit(1)

    goal = MoveGroup.Goal()
    goal.request.allowed_planning_time = 10.0
    goal.request.group_name = 'arm'
    goal.request.max_acceleration_scaling_factor = 1.0
    goal.request.max_velocity_scaling_factor = 1.0
    goal.request.workspace_parameters.header.frame_id = 'body'
    goal.request.workspace_parameters.max_corner.x = \
    goal.request.workspace_parameters.max_corner.y = \
    goal.request.workspace_parameters.max_corner.z = 5.0
    goal.request.workspace_parameters.min_corner.x = \
    goal.request.workspace_parameters.min_corner.y = \
    goal.request.workspace_parameters.min_corner.z = 5.0

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'body'
    goal_pose.header.stamp = node.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.6
    goal_pose.pose.position.y = 0.3
    goal_pose.pose.position.z = 0.4
    goal_pose.pose.orientation.w = 1.0


    goal_constraint = Constraints()
    goal_position_constraint = PositionConstraint()
    goal_orientation_constraint = OrientationConstraint()

    goal_position_constraint.header.frame_id = 'body'
    goal_position_constraint.header.stamp = node.get_clock().now().to_msg()
    goal_position_constraint.link_name = 'arm0_hand'
    goal_position_constraint.weight = 1.0

    goal_position_region = SolidPrimitive()
    goal_position_region.dimensions = [0.01]
    goal_position_region.type = SolidPrimitive.SPHERE
    goal_position_constraint.constraint_region.primitives.append(goal_position_region)
    goal_position_constraint.constraint_region.primitive_poses.append(goal_pose.pose)

    goal_orientation_constraint.header = goal_position_constraint.header
    goal_orientation_constraint.link_name = goal_position_constraint.link_name
    goal_orientation_constraint.orientation = goal_pose.pose.orientation
    goal_orientation_constraint.parameterization = OrientationConstraint.ROTATION_VECTOR
    goal_orientation_constraint.absolute_x_axis_tolerance = 0.01
    goal_orientation_constraint.absolute_y_axis_tolerance = 0.01
    goal_orientation_constraint.absolute_z_axis_tolerance = 0.01
    goal_orientation_constraint.weight = 1.0

    goal_constraint.position_constraints.append(goal_position_constraint)
    goal_constraint.orientation_constraints.append(goal_orientation_constraint)

    goal.request.goal_constraints.append(goal_constraint)

    def feedback_callback(feedback_msg):
        nonlocal node
        node.get_logger().info(f'Feedback:\n{feedback_msg.feedback}')

    node.get_logger().info('Sending goal')
    fut = action_client.send_goal_async(goal, feedback_callback=feedback_callback)
    rclpy.spin_until_future_complete(node, fut)
    goal_handle = fut.result()
    result_fut = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_fut)
    node.get_logger().info('Action complete')

if __name__ == '__main__':
    rclpy.init()
    main()