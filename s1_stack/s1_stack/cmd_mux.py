#!/usr/bin/env python3
# cmd_mux: 조이스틱 버튼으로 주행 모드(수동/자동/정지)를 고르고, 그 모드에 맞는 명령 하나를
# /cmd_out으로 내보내는 "최종 교통경찰" 노드. 이 뒤에는 serial_sender가 받아서 아두이노로 보냄.
#   /cmd_manual (joy_to_twist가 보냄, 조이스틱 수동 조작)  ─┐
#   /cmd_auto   (decision_auto가 보냄, 자율주행 최종 명령) ─┼→ cmd_mux → /cmd_out
#   /joy (조이스틱 버튼, 모드 전환용)                       ─┘
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

# 주행 모드 3가지. /drive_mode 토픽으로도 발행돼서 다른 노드/모니터링에서 현재 모드를 알 수 있음
MODE_MANUAL = 0  # 조이스틱으로 직접 운전
MODE_AUTO = 1    # 자율주행 (decision_auto가 보낸 명령 그대로 사용)
MODE_STOP = 2    # 정지 (기본 시작 모드)

def twist_zero():
    """속도 0, 조향 0인 정지 명령. 모드 전환 직후나 정지 상태에서 반복 사용"""
    t = Twist()
    t.linear.x = 0.0
    t.angular.z = 0.0
    return t

class CmdMux(Node):
    def __init__(self):
        super().__init__('cmd_mux')

        self.declare_parameter('btn_auto', 0)      # A
        self.declare_parameter('btn_manual', 1)    # B
        self.declare_parameter('btn_stop', 3)      # Y (Xbox 패드에서 3번 = Y, 10/17 확인. X는 2번)
        self.declare_parameter('initial_mode', MODE_STOP)
        self.declare_parameter('manual_timeout_s', 0.5)
        self.declare_parameter('auto_timeout_s', 0.3)
        self.declare_parameter('switch_stop_s', 0.2)

        self.btn_auto = int(self.get_parameter('btn_auto').value)
        self.btn_manual = int(self.get_parameter('btn_manual').value)
        self.btn_stop = int(self.get_parameter('btn_stop').value)
        self.mode = int(self.get_parameter('initial_mode').value)
        self.manual_timeout = float(self.get_parameter('manual_timeout_s').value)
        self.auto_timeout = float(self.get_parameter('auto_timeout_s').value)
        self.switch_stop_s = float(self.get_parameter('switch_stop_s').value)

        self.prev_buttons = []  # 직전 프레임의 버튼 상태 (버튼을 "누르는 순간"만 잡기 위한 비교용)
        self.hold_until = 0.0   # 모드가 바뀐 직후 switch_stop_s초 동안은 무조건 정지(급출발 방지)
        self.stop_until = 0.0  # [추가] STOP 모드 시 신호 지속 시간을 위한 변수
        self.last_manual = twist_zero()  # 최근 수신한 /cmd_manual 값과 받은 시각
        self.last_manual_t = 0.0
        self.last_auto = twist_zero()    # 최근 수신한 /cmd_auto 값과 받은 시각
        self.last_auto_t = 0.0

        self.pub_cmd = self.create_publisher(Twist, '/cmd_out', 10)
        self.pub_mode = self.create_publisher(UInt8, '/drive_mode', 10)

        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_subscription(Twist, '/cmd_manual', self.on_manual, 10)
        self.create_subscription(Twist, '/cmd_auto', self.on_auto, 10)

        self.timer = self.create_timer(0.05, self.on_timer)
        self.publish_mode()

    def publish_mode(self):
        m = UInt8()
        m.data = int(self.mode)
        self.pub_mode.publish(m)

    def on_joy(self, msg: Joy):
        """조이스틱 버튼 입력으로 모드 전환. 버튼을 누르는 그 순간(0→1로 바뀌는 프레임)에만 반응해서
        누르고 있는 동안 계속 반복 전환되는 걸 막음"""
        if len(self.prev_buttons) != len(msg.buttons):
            self.prev_buttons = [0] * len(msg.buttons)

        def rising(btn_idx):
            """btn_idx번 버튼이 이번 프레임에 막 눌렸는지(직전엔 0, 지금은 1)"""
            if not (0 <= btn_idx < len(msg.buttons)): return False
            return msg.buttons[btn_idx] == 1 and self.prev_buttons[btn_idx] == 0

        new_mode = None
        # 여러 버튼이 동시에 눌려도 정지가 최우선
        if rising(self.btn_stop): new_mode = MODE_STOP
        elif rising(self.btn_manual): new_mode = MODE_MANUAL
        elif rising(self.btn_auto): new_mode = MODE_AUTO

        if new_mode is not None and new_mode != self.mode:
            self.pub_cmd.publish(twist_zero())
            self.hold_until = time.time() + self.switch_stop_s
            
            # [추가] STOP 모드로 진입할 때만 1초 타이머 작동
            if new_mode == MODE_STOP:
                self.stop_until = time.time() + 1.0
            
            self.mode = new_mode
            self.publish_mode()

        self.prev_buttons = list(msg.buttons)

    def on_manual(self, msg: Twist):
        self.last_manual = msg
        self.last_manual_t = time.time()

    def on_auto(self, msg: Twist):
        self.last_auto = msg
        self.last_auto_t = time.time()

    def on_timer(self):
        """20Hz로 실제 /cmd_out을 내보내는 루프. 현재 모드에 따라 어떤 소스를 쓸지 결정"""
        now = time.time()

        # STOP 모드 처리
        if self.mode == MODE_STOP:
            if now < self.stop_until:
                # 1초가 지나기 전에는 계속 0, 0 신호 전송 (중앙 정렬 유도)
                self.pub_cmd.publish(twist_zero())
            else:
                # 1초가 지나면 아무것도 발행하지 않음 (Topic Silent)
                # 아두이노는 신호가 끊긴 것을 감지하고 1초 뒤 Failsafe 작동
                pass
            return

        # 모드를 막 바꾼 직후(switch_stop_s초)는 무조건 정지 — 이전 모드의 조향/속도가 그대로 튀어나가는 것 방지
        if now < self.hold_until:
            self.pub_cmd.publish(twist_zero())
            return

        if self.mode == MODE_AUTO:
            # 자율주행 명령(/cmd_auto)이 auto_timeout초 넘게 안 오면(decision_auto가 멈췄거나 끊김)
            # 그대로 자율주행을 믿지 않고 수동 모드로 강제 전환 — 조이스틱이 직접 멈추거나 조작해야 함
            if now - self.last_auto_t > self.auto_timeout:
                self.pub_cmd.publish(twist_zero())
                self.mode = MODE_MANUAL
                self.publish_mode()
                return
            self.pub_cmd.publish(self.last_auto)
            return

        # MODE_MANUAL: 수동 명령이 끊기면(조이스틱 연결 끊김 등) 마지막 명령을 계속 반복하지 않고 정지
        if now - self.last_manual_t >= self.manual_timeout:
            self.pub_cmd.publish(twist_zero())
            return
        self.pub_cmd.publish(self.last_manual)

def main():
    rclpy.init()
    node = CmdMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
