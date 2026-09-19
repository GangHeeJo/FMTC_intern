#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class MotionDecision(Node):
    def __init__(self):
        super().__init__('decision_auto')

        # 상태 변수 초기화
        self.stop_by_light = False
        self.stop_by_crosswalk = False
        self.current_lane_cmd = Twist()
        
        # 추가: 장애물 회피 명령 저장 변수 및 수신 시간
        self.current_obs_cmd = Twist()
        self.last_obs_time = 0.0

        # Subscribers
        self.sub_lane = self.create_subscription(Twist, '/cmd_lane', self.lane_cb, 10)
        self.sub_light = self.create_subscription(Bool, '/light_stop', self.light_cb, 10)
        self.sub_cross = self.create_subscription(Bool, '/cross_stop', self.cross_cb, 10)
        
        # 추가: 장애물 회피 명령 구독 (최우선순위)
        self.sub_obs = self.create_subscription(Twist, '/cmd_obs', self.obs_cb, 10)

        # Publisher: 최종 제어 명령
        self.pub_auto = self.create_publisher(Twist, '/cmd_auto', 10)

        # 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.publish_decision)

    def lane_cb(self, msg):
        self.current_lane_cmd = msg

    def light_cb(self, msg):
        self.stop_by_light = msg.data

    def cross_cb(self, msg):
        self.stop_by_crosswalk = msg.data

    # 추가: 장애물 회피 콜백
    def obs_cb(self, msg):
        self.current_obs_cmd = msg
        self.last_obs_time = self.get_clock().now().nanoseconds / 1e9

    def publish_decision(self):
        msg = Twist()
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # [우선순위 1] 장애물 회피 명령 확인 (최근 0.2초 이내 수신된 경우만 유효)
        if (current_time - self.last_obs_time) < 0.2:
            msg = self.current_obs_cmd
            self.get_logger().info('PRIORITY: OBSTACLE EVASION ACTIVE')
        
        # [우선순위 2] 정지 조건 확인 (신호등 OR 횡단보도)
        elif self.stop_by_light or self.stop_by_crosswalk:
            msg.angular.z = self.current_lane_cmd.angular.z
            msg.linear.x = 0.0
            self.get_logger().info('STOP SIGNAL ACTIVE')
            
        # [우선순위 3] 일반 주행 (차선 인지)
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
