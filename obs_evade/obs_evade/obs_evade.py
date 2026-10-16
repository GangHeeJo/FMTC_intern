#!/usr/bin/env python3
"""
ROS1 버전(결과보고서 PDF 부록 lidar_cluster_pub.py + lidar_lanechange_flag.py +
test_traffic_decision.py의 장애물 회피 부분)을 ROS2로 포팅.

원본과 다른 점(의도적):
- ROS1은 회전 중 while+time.sleep()으로 콜백을 블로킹함 -> ROS2에서는 non-blocking
  타이머 상태머신으로 변경 (블로킹하면 그 몇 초 동안 노드 전체가 멈춤).
- 아두이노 프로토콜이 ROS1(정수 코드 48/64/56...)과 다름(S1.ino는 Twist 기반
  TH:/STN:) -> 회전 명령은 Twist(linear.x/angular.z)로 발행.
- 3개 노드(cluster_pub/lanechange_flag/traffic_decision)로 나뉘어 있던 것을
  이 repo의 기존 obs_evade.py 관례대로 단일 노드로 합침.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
from sklearn.cluster import DBSCAN
import time


class ObstacleEvasionNode(Node):
    def __init__(self):
        super().__init__('obs_evade')

        # --- 거리 구간별 DBSCAN 파라미터 (ROS1 lidar_cluster_pub.py 기본값, mm -> m 환산) ---
        self.declare_parameter('threshold_short_m', 0.6)
        self.declare_parameter('threshold_mid_m', 3.0)
        self.declare_parameter('threshold_far_m', 5.0)
        self.declare_parameter('eps_short_m', 0.1)
        self.declare_parameter('eps_mid_m', 0.15)
        self.declare_parameter('eps_far_m', 0.2)
        self.declare_parameter('min_samples_short', 50)
        self.declare_parameter('min_samples_mid', 30)
        self.declare_parameter('min_samples_far', 20)
        # RPLidar 신호 품질(intensities) 필터. sllidar_ros2가 이 필드를 채워주는지
        # 이 하드웨어에서 확인 안 됨 -> 기본 0(미사용). 확인되면 ROS1처럼 40 정도로.
        self.declare_parameter('min_quality', 0.0)

        # --- 장애물 확정 조건 (lidar_lanechange_flag.py의 obstacle_detected) ---
        self.declare_parameter('obstacle_min_dist_m', 0.8)
        self.declare_parameter('obstacle_max_dist_m', 1.9)
        self.declare_parameter('obstacle_half_angle_deg', 18.0)  # cos(pi/10)
        self.declare_parameter('obstacle_y_half_width_m', 0.8)
        self.declare_parameter('obstacle_min_points', 50)
        self.declare_parameter('obstacle_max_points', 200)
        self.declare_parameter('confirm_frames', 3)  # ROS1: count > 2

        # --- y_coord(틀어진 정도) 계산용 조건 ---
        self.declare_parameter('yoffset_min_dist_m', 1.0)
        self.declare_parameter('yoffset_max_dist_m', 2.1)
        self.declare_parameter('yoffset_x_max_m', -1.0)  # x < 이 값(전방)
        self.declare_parameter('yoffset_y_half_width_m', 1.0)
        self.declare_parameter('yoffset_min_points', 10)

        # --- 회피 기동 ---
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_steer', 1000.0)
        # ROS1은 회전 중 forward(255,255) 풀파워 사용 -> 기본 1.0(풀파워)
        self.declare_parameter('maneuver_speed_scale', 1.0)
        # ROS1: time_left_tilt=(1.8-y)*11/25.5, time_right_tilt=(1.9+y)*11/25.5
        self.declare_parameter('left_time_base_s', 1.8)
        self.declare_parameter('right_time_base_s', 1.9)
        self.declare_parameter('time_y_coeff', 11.0 / 25.5)
        # 기동 끝난 뒤 재감지 막는 시간(ROS1: 30Hz*10프레임 ~= 0.33s)
        self.declare_parameter('pause_duration_s', 0.33)

        p = self.get_parameter
        self.threshold_short = p('threshold_short_m').value
        self.threshold_mid = p('threshold_mid_m').value
        self.threshold_far = p('threshold_far_m').value
        self.eps_short = p('eps_short_m').value
        self.eps_mid = p('eps_mid_m').value
        self.eps_far = p('eps_far_m').value
        self.min_samples_short = int(p('min_samples_short').value)
        self.min_samples_mid = int(p('min_samples_mid').value)
        self.min_samples_far = int(p('min_samples_far').value)
        self.min_quality = p('min_quality').value

        self.obstacle_min_dist = p('obstacle_min_dist_m').value
        self.obstacle_max_dist = p('obstacle_max_dist_m').value
        self.obstacle_cos_half_angle = np.cos(np.radians(p('obstacle_half_angle_deg').value))
        self.obstacle_y_half_width = p('obstacle_y_half_width_m').value
        self.obstacle_min_points = p('obstacle_min_points').value
        self.obstacle_max_points = p('obstacle_max_points').value
        self.confirm_frames = int(p('confirm_frames').value)

        self.yoffset_min_dist = p('yoffset_min_dist_m').value
        self.yoffset_max_dist = p('yoffset_max_dist_m').value
        self.yoffset_x_max = p('yoffset_x_max_m').value
        self.yoffset_y_half_width = p('yoffset_y_half_width_m').value
        self.yoffset_min_points = p('yoffset_min_points').value

        self.MAX_SPEED = p('max_speed').value
        self.MAX_STEER = p('max_steer').value
        self.maneuver_speed_scale = p('maneuver_speed_scale').value
        self.left_time_base = p('left_time_base_s').value
        self.right_time_base = p('right_time_base_s').value
        self.time_y_coeff = p('time_y_coeff').value
        self.pause_duration = p('pause_duration_s').value

        # --- 상태 ---
        self.confirm_count = 0
        self.state = 'IDLE'  # IDLE -> MANEUVER -> PAUSE -> IDLE
        self.lane_state = 'LANE2'  # LANE2=원래 차선(우측), LANE1=회피 중(좌측)
        self.maneuver_dir = 1.0  # +1=좌회전, -1=우회전
        self.maneuver_duration = 0.0
        self.phase_start_time = 0.0

        self.cmd_obs_pub = self.create_publisher(Twist, '/cmd_obs', 10)
        self.flag_pub = self.create_publisher(Bool, 'lane_change_flag', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # 20Hz: s1_stack 쪽 다른 제어 루프와 동일한 주기
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Obstacle Evasion Node (DBSCAN, ROS1 포팅) Started')

    # ---------- 감지 ----------

    def _cluster_range(self, x, y, eps, min_samples):
        """한 거리 구간의 점들을 DBSCAN으로 군집화, (center, point_count) 리스트 반환."""
        if len(x) < min_samples:
            return []
        coords = np.column_stack((x, y))
        labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(coords)
        results = []
        for label in np.unique(labels[labels >= 0]):
            pts = coords[labels == label]
            results.append((pts.mean(axis=0), len(pts)))
        return results

    def scan_callback(self, msg):
        # 기동 중이거나 쿨다운 중이면 새로 판단하지 않음 (ROS1의 lidar_detect_pause와 동일 의도)
        if self.state != 'IDLE':
            return

        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        valid = (ranges > 0.1) & (ranges < self.threshold_far) & np.isfinite(ranges)
        if self.min_quality > 0 and len(msg.intensities) == len(ranges):
            valid &= np.array(msg.intensities) >= self.min_quality

        ranges, angles = ranges[valid], angles[valid]
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        short_idx = ranges < self.threshold_short
        mid_idx = (ranges >= self.threshold_short) & (ranges < self.threshold_mid)
        far_idx = (ranges >= self.threshold_mid) & (ranges < self.threshold_far)

        clusters = []
        clusters += self._cluster_range(x[short_idx], y[short_idx], self.eps_short, self.min_samples_short)
        clusters += self._cluster_range(x[mid_idx], y[mid_idx], self.eps_mid, self.min_samples_mid)
        clusters += self._cluster_range(x[far_idx], y[far_idx], self.eps_far, self.min_samples_far)

        obstacle_detected = False
        for (cx, cy), num in clusters:
            dist = float(np.hypot(cx, cy))
            if not (self.obstacle_min_dist <= dist <= self.obstacle_max_dist):
                continue
            if dist > 0 and abs(cx) / dist <= self.obstacle_cos_half_angle:
                continue
            if not (-self.obstacle_y_half_width < cy < self.obstacle_y_half_width):
                continue
            if not (self.obstacle_min_points <= num <= self.obstacle_max_points):
                continue
            obstacle_detected = True
            break

        if obstacle_detected:
            self.confirm_count += 1
        else:
            self.confirm_count = 0

        if self.confirm_count >= self.confirm_frames:
            self.confirm_count = 0
            y_coord = self._compute_y_coord(clusters)
            self._start_maneuver(y_coord)

    def _compute_y_coord(self, clusters):
        """차가 장애물 대비 얼마나 틀어졌는지(y) - 가장 y가 큰 유효 군집 사용."""
        best_y = 0.0
        best_found = False
        for (cx, cy), num in clusters:
            dist = float(np.hypot(cx, cy))
            if not (self.yoffset_min_dist <= dist <= self.yoffset_max_dist):
                continue
            if not (cx < self.yoffset_x_max):
                continue
            if not (-self.yoffset_y_half_width < cy < self.yoffset_y_half_width):
                continue
            if num < self.yoffset_min_points:
                continue
            if not best_found or cy > best_y:
                best_y, best_found = cy, True
        return best_y if best_found else 0.0

    # ---------- 회피 기동 ----------

    def _start_maneuver(self, y_coord):
        if self.lane_state == 'LANE2':
            # 원래 차선 -> 좌회전해서 옆 차선으로
            self.maneuver_dir = 1.0
            self.maneuver_duration = max(0.0, (self.left_time_base - y_coord) * self.time_y_coeff)
        else:
            # 옆 차선 -> 우회전해서 원래 차선으로 복귀
            self.maneuver_dir = -1.0
            self.maneuver_duration = max(0.0, (self.right_time_base + y_coord) * self.time_y_coeff)

        self.state = 'MANEUVER'
        self.phase_start_time = time.time()
        self.get_logger().warn(
            f'Obstacle confirmed! lane={self.lane_state} y_coord={y_coord:.3f} '
            f'dir={"LEFT" if self.maneuver_dir > 0 else "RIGHT"} duration={self.maneuver_duration:.2f}s')

    def control_loop(self):
        flag_msg = Bool()
        flag_msg.data = (self.lane_state == 'LANE1')
        self.flag_pub.publish(flag_msg)

        if self.state == 'MANEUVER':
            elapsed = time.time() - self.phase_start_time
            if elapsed < self.maneuver_duration:
                twist = Twist()
                twist.linear.x = self.MAX_SPEED * self.maneuver_speed_scale
                twist.angular.z = self.maneuver_dir * self.MAX_STEER
                self.cmd_obs_pub.publish(twist)
            else:
                self.lane_state = 'LANE1' if self.lane_state == 'LANE2' else 'LANE2'
                self.state = 'PAUSE'
                self.phase_start_time = time.time()
                self.get_logger().info(f'Maneuver finished. lane_state={self.lane_state}')
        elif self.state == 'PAUSE':
            if time.time() - self.phase_start_time >= self.pause_duration:
                self.state = 'IDLE'
        # IDLE/PAUSE: /cmd_obs에 아무것도 발행하지 않음
        # -> decision_auto가 0.2s 뒤 자동으로 차선 추종으로 되돌아감


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleEvasionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
