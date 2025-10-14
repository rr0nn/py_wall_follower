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
        # Parameters (SMOOTHER TURNS — defaults only; logic unchanged)
        # ------------------------------
        self.declare_parameters('', [
            # Wall following
            ('target_wall_dist', 0.40),
            ('wall_tolerance',   0.13),     # wider band → fewer micro-corrections
            ('follow_side',      'left'),

            # Speeds
            ('v_straight',       0.18),
            ('v_turn',           0.13),     # slower while turning
            ('v_slow',           0.10),

            # Angular gains (softer turning)
            ('w_soft',           0.40),
            ('w_med',            0.60),

            # Obstacle handling
            ('front_block',      0.58),     # turn away a bit later
            ('no_wall',          1.00),     # avoid aggressive search
            ('corner_threshold', 0.50),     # less overreaction to diagonals

            # Sector processing
            ('sector_width',     10),       # narrower sector → less jitter
            ('min_valid_range',  0.05),
            ('max_valid_range',  4.0),

            # Control loop
            ('timer_dt',         0.01),     # 100 Hz

            # Debug
            ('debug_output',     False),

            # Start/finish detection
            ('start_enter_r',    0.30),
            ('start_exit_r',     0.40),
        ])

        # ------------------------------
        # State
        # ------------------------------
        self.sector_distances = [float('inf')] * 12
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
        qos = QoSProfile(depth=10)

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
        """Process laser scan into 12 sectors (min distance per sector)."""
        sector_width = int(self.p('sector_width'))

        for i in range(12):
            center_angle = i * 30.0
            self.sector_distances[i] = self._get_sector_min(center_angle, sector_width, msg)

        self.have_scan = True

    def _get_sector_min(self, center_angle_deg: float, half_width_deg: int, msg: LaserScan) -> float:
        """Get minimum distance in a sector."""
        min_dist = msg.range_max # actual value 3.5
        
        for offset in range(-half_width_deg, half_width_deg + 1): # -15 to 15 degrees
            angle_deg = int((center_angle_deg + offset) % 360.0)  # normalize to positive value
            
            if 0 <= angle_deg < len(msg.ranges): # make sure it is within range of < 360
                r = msg.ranges[angle_deg]
                if r > 0:
                    min_dist = min(min_dist, r)
                min_dist = min(min_dist, r)

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

        if self.p('debug_output'):
            self.get_logger().info(
                f'[ODOM] Dist from start: {dist:.3f}m | exit_r: {self.p("start_exit_r"):.2f}m | '
                f'enter_r: {self.p("start_enter_r"):.2f}m | moving: {self._start_moving}',
                throttle_duration_sec=3.0
            )

        if self._start_moving:
            if dist > self.p('start_exit_r'):
                self._start_moving = False
                self.get_logger().info(f'[ODOM] ✓ Left start area (dist: {dist:.3f}m > {self.p("start_exit_r"):.2f}m)')
        else:
            if dist < self.p('start_enter_r'):
                self.get_logger().info(f'[ODOM] ✓ COMPLETED - Returned to start (dist: {dist:.3f}m < {self.p("start_enter_r"):.2f}m)')
                self.near_start = True

    # ==============================
    # CONTROL LOGIC (unchanged)
    # ==============================

    def control_loop(self):
        """Main control loop — logic unchanged."""
        if self.near_start:
            self.publish_cmd(0.0, 0.0)
            self.timer.cancel()
            rclpy.shutdown()
            return

        if not self.have_scan:
            self.publish_cmd(0.0, 0.0)
            if self.p('debug_output'):
                self.get_logger().warn('[MAIN] Waiting for scan data...', throttle_duration_sec=2.0)
            return

        S = Sector
        follow_left = str(self.p('follow_side')).lower() == 'left'

        front = self.sector_distances[S.FRONT]

        if follow_left:
            side_est = min(self.sector_distances[S.FRONT_LEFT], self.sector_distances[S.LEFT_FRONT])
            front_diag = self.sector_distances[S.FRONT_LEFT]
            opposite_diag = self.sector_distances[S.FRONT_RIGHT]
        else:
            side_est = min(self.sector_distances[S.FRONT_RIGHT], self.sector_distances[S.RIGHT_FRONT])
            front_diag = self.sector_distances[S.FRONT_RIGHT]
            opposite_diag = self.sector_distances[S.FRONT_LEFT]

        target = self.p('target_wall_dist')
        tolerance = self.p('wall_tolerance')
        front_block = self.p('front_block')
        no_wall = self.p('no_wall')
        corner_thresh = self.p('corner_threshold')

        v_straight = self.p('v_straight')
        v_turn = self.p('v_turn')
        v_slow = self.p('v_slow')
        w_soft = self.p('w_soft')
        w_med = self.p('w_med')

        turn_sign = 1.0 if follow_left else -1.0

        # 1) Front blocked -> turn away
        if math.isfinite(front) and front < front_block:
            if self.p('debug_output'):
                self.get_logger().info(f'[1-BLOCKED] Front: {front:.2f}m < {front_block:.2f}m - turning away')
            self.publish_cmd(0.0, -turn_sign * w_med)

        # 2) No wall on the side -> search by drifting toward wall
        elif side_est > no_wall:
            if self.p('debug_output'):
                self.get_logger().info(f'[2-NO_WALL] Side: {side_est:.2f}m > {no_wall:.2f}m - searching', throttle_duration_sec=1.0)
            self.publish_cmd(v_turn, turn_sign * w_soft)

        # 3) Too close to wall -> correction away + slower
        elif side_est < target - tolerance:
            if self.p('debug_output'):
                self.get_logger().info(f'[3-TOO_CLOSE] Side: {side_est:.2f}m < {target-tolerance:.2f}m - moving away')
            self.publish_cmd(v_slow, -turn_sign * w_soft)

        # 4) Too far from wall -> correction toward wall
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

        # 7) Cruise
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