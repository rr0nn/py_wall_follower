#!/usr/bin/env python3
import math
from enum import IntEnum
from collections import deque

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


class ExponentialFilter:
    """Exponential moving average filter"""
    def __init__(self, alpha=0.3):
        self.alpha = alpha  # Smoothing factor (0-1, lower = more smoothing)
        self.value = None
    
    def update(self, new_value):
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value
    
    def get(self):
        return self.value if self.value is not None else float('inf')


class VelocityRamper:
    """Smooth velocity ramping to avoid jerky motion"""
    def __init__(self, max_accel=0.5, max_angular_accel=1.5, dt=0.01):
        self.max_accel = max_accel  # m/s²
        self.max_angular_accel = max_angular_accel  # rad/s²
        self.dt = dt
        self.current_v = 0.0
        self.current_w = 0.0
    
    def ramp(self, target_v, target_w):
        """Apply acceleration limits to reach target velocities"""
        # Linear velocity ramping
        v_diff = target_v - self.current_v
        max_v_change = self.max_accel * self.dt
        v_change = max(min(v_diff, max_v_change), -max_v_change)
        self.current_v += v_change
        
        # Angular velocity ramping
        w_diff = target_w - self.current_w
        max_w_change = self.max_angular_accel * self.dt
        w_change = max(min(w_diff, max_w_change), -max_w_change)
        self.current_w += w_change
        
        return self.current_v, self.current_w


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # ------------------------------
        # Parameters
        # ------------------------------
        self.declare_parameters('', [
            # Wall following
            ('target_wall_dist', 0.40),      # target wall following distance
            ('wall_tolerance',   0.10),
            ('follow_side',      'left'),    # follow left or right wall
            
            # Speed limits
            ('v_max',            0.30),      # maximum linear velocity
            ('v_turn',           0.25),      # turning speed
            ('v_slow',           0.15),      # slow speed when too close
            ('w_max',            0.80),      # maximum angular velocity
            
            # Proportional control gains
            ('kp_angular',       1.2),       # P gain for angular correction
            ('kp_linear',        0.3),       # P gain for linear speed reduction near obstacles
            
            # Obstacle handling
            ('front_block',      0.60),
            ('front_slow',       0.80),      # start slowing down at this distance
            ('no_wall',          1.20),
            ('corner_threshold', 0.60),      # front corner threshold
            ('corner_lookahead', 0.70),      # early corner detection
            
            # Sector processing
            ('sector_width',     15),
            ('min_valid_range',  0.05),
            ('max_valid_range',  4.0),
            
            # Filtering
            ('sensor_filter_alpha', 0.4),    # EMA smoothing (higher = less smoothing)
            ('max_accel',           0.5),    # m/s² linear acceleration limit
            ('max_angular_accel',   1.5),    # rad/s² angular acceleration limit
            
            # Control loop
            ('timer_dt',         0.01),
            
            # Debug
            ('debug_output',     True),
            
            # Start/finish detection
            ('start_enter_r',    0.15),
            ('start_exit_r',     0.30),
        ])

        # ------------------------------
        # State
        # ------------------------------
        self.sector_distances = [float('inf')] * 12
        self.have_scan = False
        
        # Filters for smooth sensor readings
        self.sector_filters = [ExponentialFilter(self.p('sensor_filter_alpha')) for _ in range(12)]
        
        # Velocity ramper for smooth motion
        self.ramper = VelocityRamper(
            max_accel=self.p('max_accel'),
            max_angular_accel=self.p('max_angular_accel'),
            dt=self.p('timer_dt')
        )
        
        # Start/finish detection
        self._first_odom = True
        self._start_moving = True
        self._start_x = 0.0
        self._start_y = 0.0
        self.near_start = False
        
        # State tracking for smoother transitions
        self.last_state = "INIT"

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
        Process laser scan into 12 sectors with exponential filtering.
        """
        sector_width = int(self.p('sector_width'))
        
        # Process all 12 sectors
        for i in range(12):
            center_angle = i * 30.0
            raw_dist = self._get_sector_min(center_angle, sector_width, msg)
            # Apply exponential filter for smoothness
            self.sector_distances[i] = self.sector_filters[i].update(raw_dist)
        
        self.have_scan = True
        
        if self.p('debug_output'):
            S = Sector
            self.get_logger().info(
                f'[SCAN] F: {self.sector_distances[S.FRONT]:.2f} | '
                f'FL: {self.sector_distances[S.FRONT_LEFT]:.2f} | '
                f'LF: {self.sector_distances[S.LEFT_FRONT]:.2f} | '
                f'L: {self.sector_distances[S.LEFT]:.2f}',
                throttle_duration_sec=1.0
            )
    
    def _get_sector_min(self, center_angle_deg: float, half_width_deg: int, msg: LaserScan) -> float:
        """Get minimum distance in a sector."""
        min_dist = float('inf')
        found_valid = False
        
        min_range = self.p('min_valid_range')
        max_range = self.p('max_valid_range')
        
        for offset in range(-half_width_deg, half_width_deg + 1):
            angle_deg = (center_angle_deg + offset) % 360.0
            angle_rad = math.radians(angle_deg)
            
            if angle_rad > math.pi:
                angle_rad -= 2 * math.pi
            
            if angle_rad < msg.angle_min or angle_rad > msg.angle_max:
                continue
            
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
        
        if self.p('debug_output'):
            self.get_logger().info(
                f'[ODOM] Dist from start: {dist:.3f}m | State: {self.last_state}',
                throttle_duration_sec=3.0
            )
        
        if self._start_moving:
            if dist > self.p('start_exit_r'):
                self._start_moving = False
                self.get_logger().info(f'[ODOM] ✓ Left start area (dist: {dist:.3f}m)')
        else:
            if dist < self.p('start_enter_r'):
                self.get_logger().info(f'[ODOM] ✓ COMPLETED - Returned to start')
                self.near_start = True

    # ==============================
    # CONTROL LOGIC
    # ==============================
    
    def control_loop(self):
        # Check if back to start
        if self.near_start:
            self.publish_cmd(0.0, 0.0, force=True)
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
        follow_left = str(self.p('follow_side')).lower() == 'left' # 'left' or 'right'
        
        # Wall distance estimation - using min() for forward-looking detection
        if follow_left:
            # side_direct = self.sector_distances[S.LEFT]
            side_front = self.sector_distances[S.LEFT_FRONT]
            side_diag = self.sector_distances[S.FRONT_LEFT]
            side_est = min(side_diag, side_front) # Use minimum for early corner/obstacle detection
            front_diag = side_diag
            opposite_diag = self.sector_distances[S.FRONT_RIGHT]
        else:
            # side_direct = self.sector_distances[S.RIGHT]
            side_front = self.sector_distances[S.RIGHT_FRONT]
            side_diag = self.sector_distances[S.FRONT_RIGHT]
            side_est = min(side_diag, side_front) # Use minimum for early corner/obstacle detection
            front_diag = side_diag
            opposite_diag = self.sector_distances[S.FRONT_LEFT]

        front = self.sector_distances[S.FRONT]
        front_est = min(front, 1.2 * side_diag, 1.2 * opposite_diag)

        # Get parameters
        target = self.p('target_wall_dist')
        tolerance = self.p('wall_tolerance')
        front_block = self.p('front_block')
        front_slow = self.p('front_slow')
        no_wall = self.p('no_wall')
        corner_threshold = self.p('corner_threshold')
        corner_lookahead = self.p('corner_lookahead')
        
        v_max = self.p('v_max')
        v_turn = self.p('v_turn')
        v_slow = self.p('v_slow')
        w_max = self.p('w_max')
        
        kp_angular = self.p('kp_angular')
        kp_linear = self.p('kp_linear')
        
        # Turn direction
        turn_sign = 1.0 if follow_left else -1.0
        
        # Default velocities
        target_v = v_max
        target_w = 0.0
        state = "UNKNOWN"
        
        # ========== STATE MACHINE ==========
        
        # 1) Front blocked -> proportional turning away
        if (math.isfinite(front_est) and front_est < front_block):
            state = "BLOCKED"
            urgency = 1.0 - (front_est / front_block) # Proportional response: closer -> turn harder
            target_v = 0.0
            target_w = -turn_sign * w_max * (0.5 + 0.5 * urgency)
            
        # 2) Front approaching -> slow down proportionally
        elif math.isfinite(front) and front < front_slow:
            # Reduce speed based on distance to obstacle
            speed_factor = (front - front_block) / (front_slow - front_block)
            speed_factor = max(0.5, min(1.0, speed_factor))
            target_v = v_max * speed_factor
            
            # Still need to decide on angular velocity based on wall tracking
            # Check other conditions below
            if side_est > no_wall:
                state = "FRONT_SLOW+NO_WALL"
                target_v = min(target_v, v_turn)
                target_w = turn_sign * w_max * 0.6
            elif side_est < target - tolerance:
                state = "FRONT_SLOW+TOO_CLOSE"
                target_v = min(target_v, v_slow)
                error = target - side_est
                target_w = -turn_sign * min(w_max, kp_angular * error)
            elif side_est > target + tolerance:
                state = "FRONT_SLOW+TOO_FAR"
                target_v = min(target_v, v_turn)
                error = side_est - target
                target_w = turn_sign * min(w_max, kp_angular * error)
            else:
                state = "FRONT_SLOW+CRUISE"
                target_w = 0.0
        
        # 3) No wall on the side -> search with proportional drift
        elif side_est > no_wall:
            state = "NO_WALL"
            target_v = v_turn
            # Gentle drift toward wall
            target_w = turn_sign * w_max * 0.5
        
        # 4) Corner detected ahead  -> prepare to turn
        elif math.isfinite(front_diag) and front_diag < corner_lookahead: # and front < corner_threshold
            state = "CORNER_AHEAD"
            # Proportional slowing and turning
            urgency = 1.0 - (front_diag / corner_lookahead)
            target_v = v_max * (1.0 - 0.5 * urgency)
            target_w = -turn_sign * w_max * (0.4 + 0.4 * urgency)
        
        # 5) Too close to wall -> proportional correction away
        elif side_est < target - tolerance:
            state = "TOO_CLOSE"
            error = target - side_est
            # Slow down more if very close
            if error > tolerance * 1.5:
                target_v = v_slow
            else:
                target_v = v_turn
            # Proportional angular correction
            target_w = -turn_sign * min(w_max, kp_angular * error)
        
        # 6) Too far from wall -> proportional correction toward
        elif side_est > target + tolerance:
            state = "TOO_FAR"
            error = side_est - target
            target_v = v_turn
            # Proportional angular correction
            target_w = turn_sign * min(w_max, kp_angular * error)
        
        # 7) Cruise - safe distance from wall
        else:
            state = "CRUISE"
            target_v = v_max
            # Very small proportional correction to stay centered
            error = side_est - target
            target_w = turn_sign * min(w_max * 0.3, kp_angular * 0.5 * error)
        
        # Log state changes
        if state != self.last_state:
            if self.p('debug_output'):
                self.get_logger().info(f'[STATE] {self.last_state} → {state}')
            self.last_state = state
        
        # Publish with ramping for smooth motion
        self.publish_cmd(target_v, target_w)
    
    def publish_cmd(self, linear: float, angular: float, force=False):
        """Publish velocity command with ramping for smooth acceleration"""
        if force:
            # Immediate stop
            v, w = linear, angular
        else:
            # Apply ramping
            v, w = self.ramper.ramp(linear, angular)
        
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
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
            node.publish_cmd(0.0, 0.0, force=True)
        except Exception:
            pass
        
        if rclpy.ok():
            node.get_logger().info('Wall follower terminated')
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()