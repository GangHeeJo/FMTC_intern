#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap')

        # ── ROS2 파라미터 선언 (FOV 제거) ──────────────────────────
        self.declare_parameter('max_speed',           1.0)
        self.declare_parameter('max_steering_angle',  0.4)
        self.declare_parameter('car_width',           0.3)
        self.declare_parameter('min_valid_range',     1.0)
        self.declare_parameter('scan_topic',          '/scan')
        self.declare_parameter('drive_topic',         '/cmd_auto')

        self._load_params()

        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(Twist, self.drive_topic, 10)

        self.get_logger().info('Follow The Gap Node Started | 360 Full View Mode')

    def _load_params(self):
        self.max_speed          = self.get_parameter('max_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.car_width          = self.get_parameter('car_width').value
        self.min_valid_range    = self.get_parameter('min_valid_range').value
        self.scan_topic         = self.get_parameter('scan_topic').value
        self.drive_topic        = self.get_parameter('drive_topic').value

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)

        # 1. 데이터 전처리
        ranges = self._preprocess(ranges, msg)

        # 2. Safety Bubble 적용 (전체 인덱스 0 ~ N 사용)
        start_idx, end_idx = 0, len(ranges) - 1
        processed_ranges = self._apply_safety_bubble(ranges, start_idx, end_idx, msg)

        # 3. 가장 큰 Gap 탐색
        gap_start, gap_end = self._find_largest_gap(processed_ranges)

        # 4. 최적 타겟 선정
        best_idx = self._find_best_point(processed_ranges, gap_start, gap_end)

        # 5. 조향각 계산 (Radian)
        steering_angle_rad = msg.angle_min + best_idx * msg.angle_increment
        
        # 6. 제어 명령 발행 (매핑 및 고정 속도)
        self._publish_drive(steering_angle_rad)

    def _preprocess(self, ranges: np.ndarray, msg: LaserScan) -> np.ndarray:
        ranges = np.where(np.isnan(ranges), 0.0, ranges)
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
        return np.clip(ranges, 0.0, msg.range_max)

    def _apply_safety_bubble(self, ranges, start_idx, end_idx, msg):
        roi = ranges[start_idx:end_idx + 1]
        valid_mask = roi > self.min_valid_range
        if not valid_mask.any(): return ranges

        masked_roi = np.where(valid_mask, roi, np.inf)
        local_closest_idx = np.argmin(masked_roi)
        closest_idx = start_idx + local_closest_idx
        closest_dist = roi[local_closest_idx]

        # Safety Bubble 계산
        bubble_angle_rad = np.arctan2(self.car_width / 2.0, max(closest_dist, 0.05))
        bubble_radius_idx = int(bubble_angle_rad / msg.angle_increment)

        bubble_start = max(0, closest_idx - bubble_radius_idx)
        bubble_end   = min(len(ranges) - 1, closest_idx + bubble_radius_idx)
        
        output_ranges = ranges.copy()
        output_ranges[bubble_start:bubble_end + 1] = 0.0
        return output_ranges

    def _find_largest_gap(self, ranges):
        masked = (ranges > self.min_valid_range).astype(np.int8)
        diff = np.diff(np.concatenate(([0], masked, [0])))
        gap_starts, gap_ends = np.where(diff == 1)[0], np.where(diff == -1)[0]
        
        if len(gap_starts) == 0: return 0, 0
        largest = np.argmax(gap_ends - gap_starts)
        return int(gap_starts[largest]), int(gap_ends[largest])

    def _find_best_point(self, ranges, gap_start, gap_end):
        if gap_start >= gap_end: return len(ranges) // 2
        gap_ranges = ranges[gap_start:gap_end]
        gap_indices = np.arange(gap_start, gap_end)
        if gap_ranges.sum() <= 0.0: return (gap_start + gap_end) // 2
        return int(np.average(gap_indices, weights=gap_ranges))

    def _publish_drive(self, steering_angle_rad):
        # 조향 매핑 (-0.4~0.4 rad -> -1000~1000)
        steering_mapped = -(steering_angle_rad / self.max_steering_angle) * 1000.0
        final_steering = float(np.clip(steering_mapped, -1000.0, 1000.0))

        # 속도 고정 (50%)
        final_speed = self.max_speed * 0.5

        msg = Twist()
        msg.linear.x = float(final_speed)
        msg.angular.z = final_steering
        self.drive_pub.publish(msg)

        self.get_logger().info(
            f'Steer: {final_steering:.0f} | Spd: {final_speed:.2f}',
            throttle_duration_sec=0.5)

def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
