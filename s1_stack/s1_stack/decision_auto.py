#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class MotionDecision(Node):
    def __init__(self):
        super().__init__('decision_auto')
        self.declare_parameter('lane_timeout_s', 0.5)
        self.declare_parameter('signal_timeout_s', 1.0)
        self.declare_parameter('require_scan', True)
        self.declare_parameter('scan_timeout_s', 0.5)
        self.declare_parameter('allow_evasion', True)
        self.lane_timeout = float(self.get_parameter('lane_timeout_s').value)
        self.signal_timeout = float(self.get_parameter('signal_timeout_s').value)
        self.require_scan = bool(self.get_parameter('require_scan').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout_s').value)
        self.allow_evasion = bool(self.get_parameter('allow_evasion').value)
        self.last_scan_time = None
        self.scan_valid = False
        self.last_lane_time = None
        self.last_light_time = None
        self.last_cross_time = None

        # 상태 변수 초기화
        self.stop_by_light = False
        self.stop_by_crosswalk = False
        self.current_lane_cmd = Twist()
        
        # 추가: 장애물 회피 명령 저장 변수 및 수신 시간
        self.current_obs_cmd = Twist()
        self.last_obs_time = None

        # Subscribers
        self.sub_lane = self.create_subscription(Twist, '/cmd_lane', self.lane_cb, 10)
        self.sub_light = self.create_subscription(Bool, '/light_stop', self.light_cb, 10)
        self.sub_cross = self.create_subscription(Bool, '/cross_stop', self.cross_cb, 10)
        
        # 추가: 장애물 회피 명령 구독 (정지 조건 다음 우선순위)
        self.sub_obs = self.create_subscription(Twist, '/cmd_obs', self.obs_cb, 10)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)

        # Publisher: 최종 제어 명령
        self.pub_auto = self.create_publisher(Twist, '/cmd_auto', 10)

        # 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.publish_decision)

    def lane_cb(self, msg):
        self.current_lane_cmd = msg
        self.last_lane_time = time.monotonic()

    def light_cb(self, msg):
        self.stop_by_light = msg.data
        self.last_light_time = time.monotonic()

    def cross_cb(self, msg):
        self.stop_by_crosswalk = msg.data
        self.last_cross_time = time.monotonic()

    # 추가: 장애물 회피 콜백
    def obs_cb(self, msg):
        self.current_obs_cmd = msg
        self.last_obs_time = time.monotonic()

    def scan_cb(self, msg):
        self.last_scan_time = time.monotonic()
        # This checks usable input, not obstacle clearance or sensor orientation.
        self.scan_valid = (
            math.isfinite(msg.range_min) and math.isfinite(msg.range_max)
            and 0 <= msg.range_min < msg.range_max
            and math.isfinite(msg.angle_increment) and msg.angle_increment != 0
            and any(math.isfinite(r) and max(0.0, msg.range_min) < r <= msg.range_max
                    for r in msg.ranges)
        )

    def publish_decision(self):
        msg = Twist()
        now = time.monotonic()

        def fresh(received_at, timeout):
            return received_at is not None and 0 <= now - received_at < timeout

        # Require live lane and signal inputs before permitting autonomous motion.
        inputs_ready = (
            fresh(self.last_lane_time, self.lane_timeout)
            and fresh(self.last_light_time, self.signal_timeout)
            and fresh(self.last_cross_time, self.signal_timeout)
            and (not self.require_scan or (
                self.scan_valid and fresh(self.last_scan_time, self.scan_timeout)))
        )
        lane_valid = (math.isfinite(self.current_lane_cmd.linear.x)
                      and math.isfinite(self.current_lane_cmd.angular.z)
                      and self.current_lane_cmd.linear.x > 0)
        if not inputs_ready or not lane_valid or self.stop_by_light or self.stop_by_crosswalk:
            # A fresh lane command can keep the steering while throttle is zero.
            if lane_valid and fresh(self.last_lane_time, self.lane_timeout):
                msg.angular.z = self.current_lane_cmd.angular.z
        elif self.allow_evasion and fresh(self.last_obs_time, 0.2):
            msg = self.current_obs_cmd
        else:
            msg = self.current_lane_cmd

        self.pub_auto.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionDecision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
