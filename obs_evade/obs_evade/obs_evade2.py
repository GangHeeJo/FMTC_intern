#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
import time

class ObstacleEvasionNode(Node):
    def __init__(self):
        super().__init__('obs_evade')

        # 1. 파라미터 설정
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_steer', 1000.0)
        self.declare_parameter('evasion_duration', 4.0)

        self.MAX_SPEED = self.get_parameter('max_speed').value
        self.MAX_STEER = self.get_parameter('max_steer').value
        self.EVASION_TIME = self.get_parameter('evasion_duration').value

        # 상태 제어 변수
        self.is_evading = False
        self.evasion_triggered = False  # 한 번이라도 트리거 됐는지 확인
        self.flag_value = False         # False=오른쪽 차선, True=왼쪽 차선
        self.evasion_start_time = 0.0
        self.obstacle_count = 0

        # 2. Publisher & Subscriber 설정
        # 평상시 주행은 다른 노드에서 하므로, 장애물 전용 토픽 /cmd_obs 사용
        self.cmd_obs_pub = self.create_publisher(Twist, '/cmd_obs', 10)
        self.flag_pub = self.create_publisher(Bool, 'lane_change_flag', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # 제어 루프 (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Obstacle Detection Node (Sticky Flag & /cmd_obs) Started')

    def scan_callback(self, msg):
        """라이다 데이터를 받아 장애물 판단"""
        if self.is_evading:
            return

        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
    
        valid_idx = (ranges > 0.1) & (ranges < 5.0)
        x = ranges[valid_idx] * np.cos(angles[valid_idx])
        y = ranges[valid_idx] * np.sin(angles[valid_idx])

        roi_idx = (x < -1.0) & (x > -1.15) & (y > -1.0) & (y < 1.0)
    
        if np.sum(roi_idx) >= 30:
            self.obstacle_count += 1
        else:
            self.obstacle_count = 0

        if self.obstacle_count > 5:
            self.start_evasion()


    def start_evasion(self):
        """회피 기동 상태 진입"""
        self.is_evading = True
        self.evasion_triggered = True
        self.flag_value = not self.flag_value
        self.evasion_start_time = time.time()

        if self.flag_value:
            self.get_logger().warn('Obstacle Detected! Lane change: RIGHT -> LEFT')
        else:
            self.get_logger().warn('Obstacle Detected! Lane change: LEFT -> RIGHT')

    def control_loop(self):
        """요청하신 수정 로직 반영"""
        # 플래그 발행: 현재 목표 차선 상태 발행
        flag_msg = Bool()
        flag_msg.data = self.flag_value
        self.flag_pub.publish(flag_msg)

        # 2. 장애물 회피 주행 명령 발행 (/cmd_obs)
        if self.is_evading:
            current_time = time.time()
            elapsed_time = current_time - self.evasion_start_time

            if elapsed_time < self.EVASION_TIME:
                twist_msg = Twist()
                twist_msg.linear.x = self.MAX_SPEED * 0.5

                if self.flag_value:
                    twist_msg.angular.z = self.MAX_STEER
                else:
                    twist_msg.angular.z = -self.MAX_STEER

                self.cmd_obs_pub.publish(twist_msg)
            else:
                self.is_evading = False
                self.evasion_triggered = False
                self.obstacle_count = 0
                self.get_logger().info('Evasion Command Finished. Ready for next obstacle.')
        
        # 평상시(is_evading=False)에는 /cmd_obs에 아무것도 publish하지 않음

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
