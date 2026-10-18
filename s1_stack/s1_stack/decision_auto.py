#!/usr/bin/env python3
# decision_auto: 자율주행 쪽 입력 여러 개(차선 추종, 신호등/횡단보도 정지, 장애물 회피)를
# 우선순위에 따라 하나의 /cmd_auto 명령으로 합치는 노드. cmd_mux가 이걸 받아서 최종 출력함.
#   /cmd_lane (lane_trace)   ─┐
#   /light_stop, /cross_stop (sign_detect) ─┼→ decision_auto → /cmd_auto → cmd_mux
#   /cmd_obs  (obs_evade)    ─┘
# 우선순위: 1) 신호등/횡단보도 정지 > 2) 장애물 회피 > 3) 평상시 차선 추종
# (공식 설계 스펙: 비상정지 > 신호등/횡단보도 > 장애물회피 > 주차 > 차선추종. 비상정지는 cmd_mux가
#  이 노드보다 바깥쪽에서 처리하므로 여기서는 신호등부터 시작. 2026-10-18: 장애물회피가 신호등보다
#  위에 있던 순서를 스펙대로 뒤집음 — 차선변경 중 빨간불/횡단보도를 만나면 변경을 끝까지 계속하던
#  문제를 고침)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class MotionDecision(Node):
    def __init__(self):
        super().__init__('decision_auto')

        # 입력 신선도(stale input) 체크용 타임아웃
        self.declare_parameter('lane_timeout_s', 0.5)
        self.declare_parameter('signal_timeout_s', 1.0)
        # 카메라 1대로만 벤치 테스트할 때 등, sign_detect(신호등) 없이 돌려야 하면 false로
        self.declare_parameter('require_signals', True)
        self.lane_timeout = float(self.get_parameter('lane_timeout_s').value)
        self.signal_timeout = float(self.get_parameter('signal_timeout_s').value)
        self.require_signals = bool(self.get_parameter('require_signals').value)
        self.last_lane_time = None
        self.last_light_time = None
        self.last_cross_time = None

        # 상태 변수 초기화
        self.stop_by_light = False       # 빨간불이면 True (sign_detect가 보냄)
        self.stop_by_crosswalk = False   # 횡단보도 감지되면 True (sign_detect가 보냄)
        self.current_lane_cmd = Twist()  # 가장 최근 받은 차선 추종 명령

        # 추가: 장애물 회피 명령 저장 변수 및 수신 시간
        self.current_obs_cmd = Twist()  # obs_evade가 차선 변경 중일 때 보내는 명령 (평소엔 안 옴)
        self.last_obs_time = 0.0

        # Subscribers
        self.sub_lane = self.create_subscription(Twist, '/cmd_lane', self.lane_cb, 10)
        self.sub_light = self.create_subscription(Bool, '/light_stop', self.light_cb, 10)
        self.sub_cross = self.create_subscription(Bool, '/cross_stop', self.cross_cb, 10)
        
        # 추가: 장애물 회피 명령 구독 (신호등/횡단보도 정지 다음 순위, 평상시 차선추종보다는 위)
        self.sub_obs = self.create_subscription(Twist, '/cmd_obs', self.obs_cb, 10)

        # Publisher: 최종 제어 명령
        self.pub_auto = self.create_publisher(Twist, '/cmd_auto', 10)

        # 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.publish_decision)

    def lane_cb(self, msg):
        self.current_lane_cmd = msg
        self.last_lane_time = self.get_clock().now().nanoseconds / 1e9

    def light_cb(self, msg):
        self.stop_by_light = msg.data
        self.last_light_time = self.get_clock().now().nanoseconds / 1e9

    def cross_cb(self, msg):
        self.stop_by_crosswalk = msg.data
        self.last_cross_time = self.get_clock().now().nanoseconds / 1e9

    # 추가: 장애물 회피 콜백
    def obs_cb(self, msg):
        self.current_obs_cmd = msg
        self.last_obs_time = self.get_clock().now().nanoseconds / 1e9

    def publish_decision(self):
        """10Hz 타이머. 지금까지 콜백으로 쌓인 값들을 보고 최종 /cmd_auto 하나를 결정해서 내보냄"""
        msg = Twist()
        current_time = self.get_clock().now().nanoseconds / 1e9

        def fresh(received_at, timeout):
            """해당 토픽이 timeout초 이내에 수신됐는지(= 그 노드가 살아서 값을 계속 보내고 있는지)"""
            return received_at is not None and 0 <= current_time - received_at < timeout

        # [안전장치] 차선/신호 입력이 끊기거나 오래됐으면, 다른 판단 하지 말고 바로 정지
        # (require_signals=false면 신호등 카메라 없이 차선만으로도 테스트 가능)
        inputs_ready = fresh(self.last_lane_time, self.lane_timeout)
        if self.require_signals:
            inputs_ready = (
                inputs_ready
                and fresh(self.last_light_time, self.signal_timeout)
                and fresh(self.last_cross_time, self.signal_timeout)
            )
        if not inputs_ready:
            msg.angular.z = self.current_lane_cmd.angular.z
            msg.linear.x = 0.0
            self.get_logger().warn('STALE INPUT - STOPPING')
            self.pub_auto.publish(msg)
            return

        # [우선순위 1] 정지 조건 확인 (신호등 OR 횡단보도) — 장애물 회피 중이어도 신호가 우선
        if self.stop_by_light or self.stop_by_crosswalk:
            msg.angular.z = self.current_lane_cmd.angular.z
            msg.linear.x = 0.0
            self.get_logger().info('STOP SIGNAL ACTIVE')

        # [우선순위 2] 장애물 회피 명령 확인 (최근 0.2초 이내 수신된 경우만 유효)
        elif (current_time - self.last_obs_time) < 0.2:
            msg = self.current_obs_cmd
            self.get_logger().info('PRIORITY: OBSTACLE EVASION ACTIVE')

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
