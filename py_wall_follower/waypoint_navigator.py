#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from pathlib import Path
import csv
import math
import time
from .landmark import marker_type

class WaypointNavigator(Node):
    """
    Reads a CSV file with columns: x, y, marker_id
    and navigates sequentially through all waypoints.
    """

    def __init__(self):
        super().__init__('waypoint_navigator')

        # --- Declare parameter for CSV path ---
        self.declare_parameter('csv_path', 'landmarks.csv')
        csv_path_param = self.get_parameter('csv_path').get_parameter_value().string_value
        if not csv_path_param:
            raise ValueError("Parameter 'csv_path' is required but not provided.")

        self.csv_path = Path(csv_path_param).expanduser().resolve()
        self.get_logger().info(f"Using waypoint file: {self.csv_path}")

        # --- Create NavigateToPose action client ---
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._wait_for_server()

        # --- Load waypoints ---
        self.waypoints = self._load_csv(self.csv_path)
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {self.csv_path}")

    # ---------- Core Nav2 communication ----------
    def _wait_for_server(self):
        """Wait until the Nav2 action server is available."""
        self.get_logger().info("Waiting for Nav2 action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Nav2 action server available.")

    def _send_goal(self, x, y, yaw_deg=0.0, frame_id='map', wait=True):
        """Send a single navigation goal to Nav2."""
        # Convert yaw to quaternion
        yaw_rad = math.radians(yaw_deg)
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f"Sending goal → ({x:.2f}, {y:.2f})")
        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2.")
            return GoalStatus.STATUS_ABORTED

        self.get_logger().info("Goal accepted, waiting for result...")

        if not wait:
            return goal_handle

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result is None:
            self.get_logger().error("No result received.")
            return GoalStatus.STATUS_ABORTED

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ Goal succeeded.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("⚠️ Goal aborted!")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("🚫 Goal canceled!")
        else:
            self.get_logger().warn(f"Unknown goal status: {status}")

        return status

    # ---------- CSV Loading ----------
    def _load_csv(self, csv_path: Path):
        """Load (x, y, marker_id) from CSV file."""
        if not csv_path.exists():
            raise FileNotFoundError(f"CSV file not found: {csv_path}")

        waypoints = []
        with csv_path.open('r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) < 3:
                    self.get_logger().warn(f"Skipping invalid row (not enough columns): {row}")
                    continue
                try:
                    x = float(row[0])
                    y = float(row[1])
                    marker_id = int(row[2])
                    waypoints.append((x, y, marker_id))
                except ValueError as e:
                    self.get_logger().warn(f"Skipping invalid row: {row} ({e})")

        return waypoints

    # ---------- Main loop ----------
    def navigate_all(self):
        """Navigate through all waypoints sequentially."""
        for i, (x, y, marker_id) in enumerate(self.waypoints, start=1):
            marker_color = marker_type[marker_id]
            self.get_logger().info(f"Navigating to waypoint {i}/{len(self.waypoints)} [Color: {marker_color}] → ({x:.2f}, {y:.2f})")
            status = self._send_goal(x, y)

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"✅ Reached waypoint {i} (Color: {marker_color}).")
            else:
                self.get_logger().warn(f"⚠️ Could not reach waypoint {i} (Color: {marker_color}), skipping...")
                
            # Wait 1 second before moving to next waypoint.
            self.get_logger().info("Waiting 1 second before next waypoint...")
            time.sleep(1.0)

        self.get_logger().info("🎯 All waypoints processed.")


def main():
    rclpy.init()
    navigator = WaypointNavigator()
    navigator.navigate_all()
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()