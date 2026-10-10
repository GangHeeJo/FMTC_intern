#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

MODE_MANUAL = 0
MODE_AUTO = 1
MODE_STOP = 2

def twist_zero():
    t = Twist()
    t.linear.x = 0.0
    t.angular.z = 0.0
    return t

class CmdMux(Node):
    def __init__(self):
        super().__init__('cmd_mux')

        self.declare_parameter('btn_auto', 0)      # A
        self.declare_parameter('btn_manual', 1)    # B
        self.declare_parameter('btn_stop', 3)      # X
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

        self.prev_buttons = []
        self.hold_until = 0.0
        self.stop_until = 0.0  # [추가] STOP 모드 시 신호 지속 시간을 위한 변수
        self.last_manual = twist_zero()
        self.last_manual_t = 0.0
        self.last_auto = twist_zero()
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
        if len(self.prev_buttons) != len(msg.buttons):
            self.prev_buttons = [0] * len(msg.buttons)

        def rising(btn_idx):
            if not (0 <= btn_idx < len(msg.buttons)): return False
            return msg.buttons[btn_idx] == 1 and self.prev_buttons[btn_idx] == 0

        new_mode = None
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
        
        if now < self.hold_until:
            self.pub_cmd.publish(twist_zero())
            return

        if self.mode == MODE_AUTO:
            if now - self.last_auto_t > self.auto_timeout:
                self.pub_cmd.publish(twist_zero())
                self.mode = MODE_MANUAL
                self.publish_mode()
                return
            self.pub_cmd.publish(self.last_auto)
            return

        # 수동 명령이 끊기면(조이스틱 연결 끊김 등) 마지막 명령을 계속 반복하지 않고 정지
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
