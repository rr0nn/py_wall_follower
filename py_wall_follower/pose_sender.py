# Testing NavigateToPose

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

def main():
    rclpy.init()
    node = rclpy.create_node('auto_goal_sender')
    action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    # Wait for Nav2 to be ready
    action_client.wait_for_server()

    # Create the goal message
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = 0.0
    goal_msg.pose.pose.position.y = 0.0
    goal_msg.pose.pose.orientation.w = 1.0  # facing forward

    # Send it
    node.get_logger().info('Sending goal to Nav2...')
    future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future)

    node.get_logger().info('Goal sent successfully!')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()