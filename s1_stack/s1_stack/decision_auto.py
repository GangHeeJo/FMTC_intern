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
from std_msgs.msg import Bool, Float32, UInt8
from geometry_msgs.msg import Twist
from collections import deque

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
        # 정지선 모드(10/19): 신호등(빨간·노란불)으로는 상단 카메라의 횡단보도 정지선(/stop_line_y, lane_trace)이
        # 화면 y ≥ stop_line_y에 왔을 때만 멈추고, 초록불이 되면 출발한 뒤 cw_pass_s 동안은 정지선을 안 봄.
        # 앞 카메라 YOLO의 횡단보도(/cross_stop)는 안 씀. 정지선에서 신호 입력이 끊겨 있으면 멈춤
        self.declare_parameter('stop_line_mode', False)
        self.declare_parameter('stop_line_y', 320.0)        # 640x480 화면. 10/19 사용자가 세운 정지 위치에서 326~330
        self.declare_parameter('stop_line_timeout_s', 0.5)
        self.declare_parameter('manual_mode', 0)           # /drive_mode 값 중 수동(B). 정지선 상태를 풂
        self.declare_parameter('cw_line_recent_s', 1.0)    # 정지선을 이 시간 안에 봤으면 횡단보도 구간(/crosswalk_active)
        # 정지선에서 출발: 초록불(sign_detect /light_green)이 이 시간 동안 계속 보이고 빨간·노란불이 아닐 때만.
        # 10/19 bag_full_195517: 서는 순간 흔들려 빨간불 확신도가 0.37까지 떨어져 light_stop이 0.3초 끊기자 빨간불에 출발함
        self.declare_parameter('go_green_s', 0.5)
        # 정지선이 이 높이까지 내려온 뒤(직전 1초에 3번 이상 잡힘) 안 보이게 되면 보닛 아래로 들어간 것 → 도착으로 봄
        # (bag_full_193634: 비스듬히 다가가며 y 256 → 301 다음 프레임부터 사라짐)
        self.declare_parameter('stop_line_near_y', 260.0)
        self.declare_parameter('cw_pass_s', 5.0)
        gp = lambda n: self.get_parameter(n).value
        self.stop_line_mode = bool(gp('stop_line_mode'))
        self.stop_line_th = float(gp('stop_line_y'))
        self.stop_line_timeout = float(gp('stop_line_timeout_s'))
        self.manual_mode = int(gp('manual_mode'))
        self.drive_mode = None
        self.cw_line_recent = float(gp('cw_line_recent_s'))
        self.go_green = float(gp('go_green_s'))
        self.green = False
        self.green_since = None
        self.last_green_time = None
        self.cw_pass = float(gp('cw_pass_s'))
        self.stop_line_near = float(gp('stop_line_near_y'))
        self.line_hist = deque(maxlen=60)   # 최근 정지선 검출 (시각, y)
        self.stop_line_y = -1.0
        self.last_stop_line_time = None
        self.last_line_seen = 0.0
        self.cw_state = 'wait'      # wait: 정지선 기다림, at_line: 정지선(신호 봄), passing: 초록불에 출발해 지나는 중
        self.cw_pass_start = 0.0
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
        self.sub_stop_line = self.create_subscription(Float32, '/stop_line_y', self.stop_line_cb, 10)
        self.sub_green = self.create_subscription(Bool, '/light_green', self.green_cb, 10)
        self.sub_mode = self.create_subscription(UInt8, '/drive_mode', self.mode_cb, 10)
        
        # 추가: 장애물 회피 명령 구독 (신호등/횡단보도 정지 다음 순위, 평상시 차선추종보다는 위)
        self.sub_obs = self.create_subscription(Twist, '/cmd_obs', self.obs_cb, 10)

        # Publisher: 최종 제어 명령
        self.pub_auto = self.create_publisher(Twist, '/cmd_auto', 10)
        # 횡단보도 구간(정지선 보임·정지선에서 대기·초록불에 지나는 중). obs_evade가 이동안 앞 장애물 감지를 쉼
        self.pub_cw_active = self.create_publisher(Bool, '/crosswalk_active', 10)

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

    def mode_cb(self, msg):
        self.drive_mode = int(msg.data)

    def green_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if msg.data and not self.green:
            self.green_since = now
        self.green = bool(msg.data)
        self.last_green_time = now

    def stop_line_cb(self, msg):
        self.stop_line_y = float(msg.data)
        self.last_stop_line_time = self.get_clock().now().nanoseconds / 1e9
        if self.stop_line_y >= 0.0:
            self.line_hist.append((self.last_stop_line_time, self.stop_line_y))

    def update_crosswalk(self, now, fresh):
        """정지선 모드 상태 갱신. 지금 정지선에서 서야 하면 True"""
        line_now = fresh(self.last_stop_line_time, self.stop_line_timeout) and self.stop_line_y >= 0.0
        if line_now:
            self.last_line_seen = now
        recent = [y for t, y in self.line_hist if now - t < 1.0]
        if self.cw_state == 'wait' and line_now and self.stop_line_y >= self.stop_line_th and len(recent) >= 2:
            self.cw_state = 'at_line'
            self.get_logger().warn(f'Stop line reached (y {self.stop_line_y:.0f})')
        elif (self.cw_state == 'wait' and not line_now and len(recent) >= 3 and self.line_hist
              and now - self.line_hist[-1][0] < self.stop_line_timeout and self.line_hist[-1][1] >= self.stop_line_near):
            self.cw_state = 'at_line'
            self.last_line_seen = now
            self.get_logger().warn(f'Stop line went under the view (last y {self.line_hist[-1][1]:.0f}) -> reached')
        if self.cw_state == 'at_line':
            # 정지선이 보닛 아래로 들어가 안 보여도 계속 정지선 상태(서 있는 동안 못 봄, 10/19). 풀리는 건 초록불이나
            # 수동 모드(차를 손으로 옮김)뿐. 예전에는 먼 곳에 정지선이 다시 보이면 풀었는데, 서 있는 동안 줄무늬 사이 가로선을
            # y 194로 잘못 보고 풀려 빨간불에 출발함(bag_full_201527 118.9초)
            if self.drive_mode == self.manual_mode:
                self.cw_state = 'wait'
                self.get_logger().warn('Manual mode at stop line -> waiting for stop line')
            elif not fresh(self.last_light_time, self.signal_timeout):
                self.get_logger().warn('At stop line, no traffic light input - STOPPING', throttle_duration_sec=1.0)
                return True
            elif self.stop_by_light:
                self.get_logger().info('At stop line, red/yellow light - STOPPING', throttle_duration_sec=1.0)
                return True
            elif not (self.green and fresh(self.last_green_time, self.signal_timeout)
                      and now - self.green_since >= self.go_green):
                self.get_logger().info('At stop line, waiting for green', throttle_duration_sec=1.0)
                return True
            else:
                self.cw_state = 'passing'
                self.cw_pass_start = now
                self.get_logger().warn(f'At stop line, green for {now - self.green_since:.1f}s -> GO')
        if self.cw_state == 'passing' and now - self.cw_pass_start > self.cw_pass:
            self.cw_state = 'wait'
        return False

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
        # 정지선 모드에서는 신호 끊김을 정지선에서만 봄(sign_detect가 1~3초씩 멈춰도 정지선 밖에서는 계속 달림, 10/17)
        if self.require_signals and not self.stop_line_mode:
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

        # 정지선 모드: 정지선에서 빨간·노란불이면 장애물 회피보다도 먼저 정지(10/19 bag_full_195127: 정지선에서 선 뒤
        # 횡단보도 너머 구조물을 장애물로 잡아 빨간불에 차선 변경을 시작함). 횡단보도 구간이면 obs_evade에 알림
        cw_stop = self.stop_line_mode and self.update_crosswalk(current_time, fresh)
        if self.stop_line_mode:
            recent_line = any(current_time - t < self.cw_line_recent for t, _ in self.line_hist)
            self.pub_cw_active.publish(Bool(data=self.cw_state != 'wait' or recent_line))

        # [우선순위 1] 정지 조건 확인 (정지선 모드: 위에서 계산한 cw_stop / 아니면 신호등 OR 횡단보도)
        # — 장애물 회피 중이어도 신호가 우선(2026-10-18: 차선변경 중 빨간불을 만나면 변경을 끝까지 계속하던 문제를 고침)
        if cw_stop or (not self.stop_line_mode and (self.stop_by_light or self.stop_by_crosswalk)):
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
