#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ── 상수 정의 ────────────────────────────────────────────────────────────────
DERIVATIVE_FILTER_ALPHA   = 0.8   # Derivative low-pass 필터 계수
STEER_RAD_LIMIT           = 0.4   # 매핑 기준이 되는 최대 조향각 (rad)
STEER_MAP_LIMIT           = 1000.0 # 아두이노 전송용 최대 스케일
LOG_THROTTLE_SEC          = 2.0   # 로그 출력 주기 (s)

class WallFollow(Node):
    """
    ROS2 Wall Following Node — 왼쪽 벽 추종 (Twist 전용)
    - 조향: -0.4~0.4 rad -> -1000~1000 매핑
    - 속도: max_velocity의 50% 고정
    """

    def __init__(self):
        super().__init__('wall_follow_node')

        # ── ROS 파라미터 선언 ────────────────────────────────────────────────
        self.declare_parameter('exec_mode',         'real')   # 'sim' | 'real'
        self.declare_parameter('kp',                1.0)
        self.declare_parameter('ki',                0.0)
        self.declare_parameter('kd',                0.0)
        self.declare_parameter('desired_distance',  1.0)
        self.declare_parameter('lookahead_dist',    1.0)
        self.declare_parameter('max_integral',      5.0)
        self.declare_parameter('max_velocity',      1.0)     # 이 값의 50%로 주행
        self.declare_parameter('min_valid_measurements', 15)
        self.declare_parameter('safety_timeout',    0.5)
        self.declare_parameter('search_width',      0.35)

        # ── 파라미터 읽기 ────────────────────────────────────────────────────
        self.exec_mode         = self.get_parameter('exec_mode').value
        self.kp                = self.get_parameter('kp').value
        self.ki                = self.get_parameter('ki').value
        self.kd                = self.get_parameter('kd').value
        self.desired_distance  = self.get_parameter('desired_distance').value
        self.lookahead_dist    = self.get_parameter('lookahead_dist').value
        self.max_integral      = self.get_parameter('max_integral').value
        self.max_velocity      = self.get_parameter('max_velocity').value
        self.min_valid_measurements = self.get_parameter('min_valid_measurements').value
        self.safety_timeout    = self.get_parameter('safety_timeout').value
        self.search_width      = self.get_parameter('search_width').value

        # ── PID 상태 변수 ────────────────────────────────────────────────────
        self.integral        = 0.0
        self.prev_error      = 0.0
        self.prev_derivative = 0.0
        self.prev_time       = self.get_clock().now()
        self.last_valid_scan_time = self.get_clock().now()

        # ── 왼쪽 벽 추종 각도 ────────────────────────────────────────────────
        self.left_perp_angle    = math.pi / 2   # 90° (b 빔)
        self.left_forward_angle = math.pi / 4   # 45° (a 빔)

        # ── Pub/Sub ──────────────────────────────────────────────────────────
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Serial 노드가 구독하는 Twist 토픽 이름으로 설정 (/cmd_vel 또는 /cmd_auto)
        self.drive_pub = self.create_publisher(Twist, '/cmd_auto', 10)

        # Safety 타이머
        self.create_timer(0.1, self._timer_safety_check)

        self.get_logger().info(f'WallFollow Initialized | Mode: {self.exec_mode}')

    def _angle_to_index(self, angle, angle_min, angle_increment, length):
        idx = int((angle - angle_min) / angle_increment)
        return max(0, min(idx, length - 1))

    def find_valid_range(self, ranges_np, angle, angle_min, angle_increment):
        """지정 각도 근방에서 가장 가까운 유효 측정값 반환"""
        n = len(ranges_np)
        center_idx = self._angle_to_index(angle, angle_min, angle_increment, n)
        half_steps = max(1, int(self.search_width / angle_increment))

        lo, hi = max(0, center_idx - half_steps), min(n, center_idx + half_steps + 1)
        segment = ranges_np[lo:hi]
        valid_mask = np.isfinite(segment)

        if not np.any(valid_mask):
            return float('inf'), angle

        valid_indices = np.where(valid_mask)[0]
        closest_in_seg = valid_indices[np.argmin(np.abs(valid_indices - (center_idx - lo)))]
        actual_idx = lo + int(closest_in_seg)
        return float(ranges_np[actual_idx]), angle_min + actual_idx * angle_increment

    def get_error(self, ranges_np, dist, angle_min, angle_increment):
        """F1TENTH 표준 기하학 공식 기반 오차 계산"""
        perp_dist, _ = self.find_valid_range(ranges_np, self.left_perp_angle, angle_min, angle_increment)
        forward_dist, _ = self.find_valid_range(ranges_np, self.left_forward_angle, angle_min, angle_increment)

        if not np.isfinite(perp_dist) or not np.isfinite(forward_dist):
            return 0.5 # 벽에서 멀어지도록 유도

        theta = abs(self.left_perp_angle - self.left_forward_angle)
        alpha = math.atan2(forward_dist * math.cos(theta) - perp_dist, forward_dist * math.sin(theta))
        
        current_dist = perp_dist * math.cos(alpha)
        predicted_dist = current_dist + self.lookahead_dist * math.sin(alpha)
        return dist - predicted_dist

    def pid_control(self, error):
        """PID 계산 및 -1000~1000 매핑 후 Twist 발행"""
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time
        if not (0.001 <= dt <= 1.0): dt = 0.1

        # PID 항 계산
        p_term = self.kp * error
        self.integral = float(np.clip(self.integral + error * dt, -self.max_integral, self.max_integral))
        i_term = self.ki * self.integral

        raw_derivative = (error - self.prev_error) / dt
        filtered_deriv = (DERIVATIVE_FILTER_ALPHA * raw_derivative + (1.0 - DERIVATIVE_FILTER_ALPHA) * self.prev_derivative)
        self.prev_derivative, self.prev_error = filtered_deriv, error
        d_term = self.kd * filtered_deriv

        # 1. 조향각(Radian) 계산 및 -1000 ~ 1000 매핑
        # 왼쪽 벽 추종: error > 0(가까움)일 때 오른쪽(-)으로 조향
        steering_angle_rad = -(p_term + i_term + d_term)
        
        # -0.4 ~ 0.4 범위를 -1000 ~ 1000으로 선형 매핑
        steering_mapped = (steering_angle_rad / STEER_RAD_LIMIT) * STEER_MAP_LIMIT
        final_steering = float(np.clip(steering_mapped, -STEER_MAP_LIMIT, STEER_MAP_LIMIT))

        # 2. 속도 결정: 최대 속도의 50% 고정
        final_speed = self.max_velocity * 0.5

        # 3. Twist 발행
        msg = Twist()
        msg.linear.x = float(final_speed)
        msg.angular.z = final_steering
        self.drive_pub.publish(msg)

        self.get_logger().info(
            f'Err: {error:.2f} | Steer: {final_steering:.0f} | Spd: {final_speed:.2f}',
            throttle_duration_sec=LOG_THROTTLE_SEC)

    def publish_stop(self, reason=''):
        msg = Twist()
        self.drive_pub.publish(msg)
        self.get_logger().warn(f'STOP | {reason}')

    def _timer_safety_check(self):
        elapsed = (self.get_clock().now() - self.last_valid_scan_time).nanoseconds / 1e9
        if elapsed > self.safety_timeout:
            self.publish_stop(f'Timeout {elapsed:.2f}s')

    def scan_callback(self, msg):
        raw = np.array(msg.ranges, dtype=np.float64)
        if self.exec_mode == 'real':
            half = len(raw) // 2
            raw = np.concatenate([raw[half:], raw[:half]])

        if np.sum(np.isfinite(raw)) < self.min_valid_measurements:
            self.publish_stop('Low valid scans')
            return

        self.last_valid_scan_time = self.get_clock().now()
        error = self.get_error(raw, self.desired_distance, msg.angle_min, msg.angle_increment)
        
        if math.isfinite(error):
            self.pid_control(error)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop('Shutdown')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
