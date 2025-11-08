#!/usr/bin/env python3

"""
Testing NavigateToPose
"""

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

def main():
    rclpy.init()
    node = rclpy.create_node('auto_goal_sender')
    action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    node.get_logger().info('Waiting for Nav2 action server...')
    action_client.wait_for_server()

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = 2.9198
    goal_msg.pose.pose.position.y = 0.1020
    goal_msg.pose.pose.orientation.w = 1.0 # facing +x

    node.get_logger().info('Sending goal...')
    send_future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_future)
    goal_handle = send_future.result()

    if not goal_handle.accepted:
        node.get_logger().error('Goal rejected.')
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info('Goal accepted, waiting for result...')
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    status = result_future.result().status

    if status == GoalStatus.STATUS_SUCCEEDED:
        node.get_logger().info('Goal succeeded (within tolerance).')
    elif status == GoalStatus.STATUS_ABORTED:
        node.get_logger().warn('Goal aborted!')
    elif status == GoalStatus.STATUS_CANCELED:
        node.get_logger().warn('Goal canceled!')
    else:
        node.get_logger().warn(f'Goal ended with unknown status: {status}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()