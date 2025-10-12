#!/usr/bin/env python3
import math
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class Sector(IntEnum):
    """12 sectors around the robot at 30° intervals"""
    FRONT        = 0     #   0°
    FRONT_LEFT   = 1     #  30°
    LEFT_FRONT   = 2     #  60°
    LEFT         = 3     #  90°
    LEFT_BACK    = 4     # 120°
    BACK_LEFT    = 5     # 150°
    BACK         = 6     # 180°
    BACK_RIGHT   = 7     # 210°
    RIGHT_BACK   = 8     # 240°
    RIGHT        = 9     # 270°
    RIGHT_FRONT  = 10    # 300°
    FRONT_RIGHT  = 11    # 330°


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # ------------------------------
        # Parameters
        # ------------------------------
        self.declare_parameters('', [
            # Wall following
            ('target_wall_dist', 0.5),      # cm
            ('wall_tolerance',   0.10),      # ±cm
            ('follow_side',      'left'),    # 'left' or 'right'
            
            # Speed limits
            ('v_straight',       0.15),      # straight line speed
            ('v_turn',           0.12),      # turning speed
            ('v_slow',           0.08),      # slow speed when too close
            ('w_soft',           0.60),      # soft angular velocity
            ('w_med',            0.70),      # medium angular velocity
            
            # Obstacle handling
            ('front_block',      0.60),      # front blocked threshold
            ('no_wall',          1.20),      # consider wall lost beyond this
            ('corner_threshold', 0.55),      # diagonal corner detection
            
            # Sector processing
            ('sector_width',     15),        # BEAM_WIDTH
            ('min_valid_range',  0.05),      # min valid LiDAR reading [m]
            ('max_valid_range',  4.0),       # max valid LiDAR reading [m]
            
            # Control loop
            ('timer_dt',         0.01),      # 10ms
            
            # Debug
            ('debug_output',     False),      # enable debug output
            
            # Start/finish detection
            ('start_enter_r',    0.30),      # return detection radius
            ('start_exit_r',     0.40),      # must leave this radius first
        ])

        # ------------------------------
        # State
        # ------------------------------
        self.sector_distances = [float('inf')] * 12  # Distance readings for 12 sectors
        self.have_scan = False
        
        # Start/finish detection
        self._first_odom = True
        self._start_moving = True
        self._start_x = 0.0
        self._start_y = 0.0
        self.near_start = False

        # ------------------------------
        # Publishers / Subscribers
        # ------------------------------
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data
        )
        
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos
        )

        # ------------------------------
        # Timer
        # ------------------------------
        self.timer = self.create_timer(self.p('timer_dt'), self.control_loop)

        self.get_logger().info(
            f'Wall follower initialized - following {self.p("follow_side")} wall at {self.p("target_wall_dist")}m'
        )

    def p(self, name: str):
        """Get parameter value"""
        return self.get_parameter(name).value

    # ==============================
    # SENSOR PROCESSING
    # ==============================
    
    def scan_callback(self, msg: LaserScan):
        """
        Process laser scan into 12 sectors (matching C++ implementation).
        Each sector contains the minimum distance in that direction.
        No smoothing - direct readings like C++ version.
        """
        sector_width = int(self.p('sector_width'))
        
        # Process all 12 sectors
        for i in range(12):
            center_angle = i * 30.0  # Each sector is 30° apart
            self.sector_distances[i] = self._get_sector_min(center_angle, sector_width, msg)
        
        self.have_scan = True
        
        # if self.p('debug_output'):
        #     S = Sector
        #     self.get_logger().info(
        #         f'[SCAN] F: {self.sector_distances[S.FRONT]:.2f} | '
        #         f'FL: {self.sector_distances[S.FRONT_LEFT]:.2f} | '
        #         f'LF: {self.sector_distances[S.LEFT_FRONT]:.2f} | '
        #         f'L: {self.sector_distances[S.LEFT]:.2f}',
        #         throttle_duration_sec=1.0
        #     )
    
    def _get_sector_min(self, center_angle_deg: float, half_width_deg: int, msg: LaserScan) -> float:
        """
        Get minimum distance in a sector.
        Samples ±half_width degrees around center_angle.
        """
        min_dist = float('inf')
        found_valid = False
        
        min_range = self.p('min_valid_range')
        max_range = self.p('max_valid_range')
        
        # Sample every degree in the sector
        for offset in range(-half_width_deg, half_width_deg + 1):
            angle_deg = (center_angle_deg + offset) % 360.0
            angle_rad = math.radians(angle_deg)
            
            # Convert to LiDAR frame (0° = forward, CCW positive)
            # Wrap to [-π, π]
            if angle_rad > math.pi:
                angle_rad -= 2 * math.pi
            
            # Check if angle is within scan range
            if angle_rad < msg.angle_min or angle_rad > msg.angle_max:
                continue
            
            # Get closest index
            idx = int(round((angle_rad - msg.angle_min) / msg.angle_increment))
            if 0 <= idx < len(msg.ranges):
                r = msg.ranges[idx]
                if min_range < r < max_range:
                    found_valid = True
                    min_dist = min(min_dist, r)
        
        return min_dist if found_valid else msg.range_max if msg.range_max > 0 else float('inf')

    def odom_callback(self, msg: Odometry):
        """Track position for start/finish detection with hysteresis"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self._first_odom:
            self._start_x = x
            self._start_y = y
            self._first_odom = False
            if self.p('debug_output'):
                self.get_logger().info(f'[ODOM] Start position set: ({x:.3f}, {y:.3f})')
            return

        dist = math.hypot(x - self._start_x, y - self._start_y)
        
        # Debug: show distance periodically
        if self.p('debug_output'):
            self.get_logger().info(
                f'[ODOM] Dist from start: {dist:.3f}m | exit_r: {self.p("start_exit_r"):.2f}m | '
                f'enter_r: {self.p("start_enter_r"):.2f}m | moving: {self._start_moving}',
                throttle_duration_sec=3.0
            )
        
        if self._start_moving:
            # Robot must first move AWAY from start position
            if dist > self.p('start_exit_r'):
                self._start_moving = False
                self.get_logger().info(f'[ODOM] ✓ Left start area (dist: {dist:.3f}m > {self.p("start_exit_r"):.2f}m)')
        else:
            # Only after leaving start can we detect a return
            if dist < self.p('start_enter_r'):
                self.get_logger().info(f'[ODOM] ✓ COMPLETED - Returned to start (dist: {dist:.3f}m < {self.p("start_enter_r"):.2f}m)')
                self.near_start = True

    # ==============================
    # CONTROL LOGIC
    # ==============================
    
    def control_loop(self):
        """
        Main control loop - matches C++ state machine logic exactly.
        Simple if-else chain with clear priorities.
        """
        # Check if finished
        if self.near_start:
            self.publish_cmd(0.0, 0.0)
            self.timer.cancel()
            rclpy.shutdown()
            return
        
        # Wait for scan data
        if not self.have_scan:
            self.publish_cmd(0.0, 0.0)
            if self.p('debug_output'):
                self.get_logger().warn('[MAIN] Waiting for scan data...', throttle_duration_sec=2.0)
            return
        
        # Get sector readings
        S = Sector
        follow_left = str(self.p('follow_side')).lower() == 'left'
        
        front = self.sector_distances[S.FRONT]
        
        # LEFT_EST = min(FRONT_LEFT, LEFT_FRONT)
        if follow_left:
            side_est = min(self.sector_distances[S.FRONT_LEFT], self.sector_distances[S.LEFT_FRONT])
            front_diag = self.sector_distances[S.FRONT_LEFT]
            opposite_diag = self.sector_distances[S.FRONT_RIGHT]
        else:
            side_est = min(self.sector_distances[S.FRONT_RIGHT], self.sector_distances[S.RIGHT_FRONT])
            front_diag = self.sector_distances[S.FRONT_RIGHT]
            opposite_diag = self.sector_distances[S.FRONT_LEFT]
        
        # Get parameters
        target = self.p('target_wall_dist')    # LEFT_TARGET = 0.35
        tolerance = self.p('wall_tolerance')    # LEFT_TOL = 0.10
        front_block = self.p('front_block')     # FRONT_BLOCK = 0.35
        no_wall = self.p('no_wall')             # NO_WALL = 1.20
        corner_thresh = self.p('corner_threshold')  # 0.55
        
        v_straight = self.p('v_straight')
        v_turn = self.p('v_turn')
        v_slow = self.p('v_slow')
        w_soft = self.p('w_soft')
        w_med = self.p('w_med')
        
        # Turn direction (left wall = turn left is positive)
        turn_sign = 1.0 if follow_left else -1.0
        
        # ========== STATE MACHINE ==========
        
        # 1) Front blocked -> gentle turn away from wall
        if math.isfinite(front) and front < front_block:
            if self.p('debug_output'):
                self.get_logger().info(f'[1-BLOCKED] Front: {front:.2f}m < {front_block:.2f}m - turning away')
            self.publish_cmd(0.0, -turn_sign * w_med)
        
        # 2) No wall on the side -> actively search by drifting toward wall
        elif side_est > no_wall:
            if self.p('debug_output'):
                self.get_logger().info(f'[2-NO_WALL] Side: {side_est:.2f}m > {no_wall:.2f}m - searching', throttle_duration_sec=1.0)
            self.publish_cmd(v_turn, turn_sign * w_soft)
        
        # 3) Too close to wall -> small correction away + slower
        elif side_est < target - tolerance:
            if self.p('debug_output'):
                self.get_logger().info(f'[3-TOO_CLOSE] Side: {side_est:.2f}m < {target-tolerance:.2f}m - moving away')
            self.publish_cmd(v_slow, -turn_sign * w_soft)
        
        # 4) Too far from wall -> small correction toward wall
        elif side_est > target + tolerance:
            if self.p('debug_output'):
                self.get_logger().info(f'[4-TOO_FAR] Side: {side_est:.2f}m > {target+tolerance:.2f}m - moving toward')
            self.publish_cmd(v_turn, turn_sign * w_soft)
        
        # 5) Front diagonal corner approaching -> turn away
        elif math.isfinite(front_diag) and front_diag < corner_thresh:
            if self.p('debug_output'):
                self.get_logger().info(f'[5-CORNER_FRONT] Front diag: {front_diag:.2f}m < {corner_thresh:.2f}m - avoiding')
            self.publish_cmd(v_turn, -turn_sign * w_soft)
        
        # 6) Opposite diagonal corner
        elif math.isfinite(opposite_diag) and opposite_diag < corner_thresh:
            if self.p('debug_output'):
                self.get_logger().info(f'[6-CORNER_OPP] Opposite diag: {opposite_diag:.2f}m < {corner_thresh:.2f}m - correcting')
            self.publish_cmd(v_turn, turn_sign * w_soft)
        
        # 7) Cruise when everything is good
        else:
            if self.p('debug_output'):
                self.get_logger().info(f'[7-CRUISE] Side: {side_est:.2f}m in [{target-tolerance:.2f}, {target+tolerance:.2f}] - straight', throttle_duration_sec=2.0)
            self.publish_cmd(v_straight, 0.0)
    
    def publish_cmd(self, linear: float, angular: float):
        """Publish velocity command"""
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_vel_pub.publish(cmd)


def main():
    rclpy.init()
    node = WallFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_cmd(0.0, 0.0)
        except Exception:
            pass
        
        if rclpy.ok():
            node.get_logger().info('Wall follower terminated')
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()