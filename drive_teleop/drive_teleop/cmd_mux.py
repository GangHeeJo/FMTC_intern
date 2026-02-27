#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

MODE_MANUAL = 0
MODE_AUTO = 1

def twist_zero():
    t = Twist()
    t.linear.x = 0.0
    t.angular.z = 0.0
    return t

class CmdMux(Node):
    def __init__(self):
        super().__init__('cmd_mux')

        # buttons
        self.declare_parameter('toggle_button', 0)  # 원하는 버튼 번호로 바꾸기
        self.declare_parameter('stop_button', 1)    # 비상정지 버튼(누르면 MANUAL+정지)
        self.declare_parameter('initial_mode', MODE_MANUAL)

        # safety timeouts
        self.declare_parameter('manual_timeout_s', 0.5)
        self.declare_parameter('auto_timeout_s', 0.3)

        self.toggle_button = int(self.get_parameter('toggle_button').value)
        self.stop_button = int(self.get_parameter('stop_button').value)
        self.mode = int(self.get_parameter('initial_mode').value)

        self.manual_timeout = float(self.get_parameter('manual_timeout_s').value)
        self.auto_timeout = float(self.get_parameter('auto_timeout_s').value)
        
        self.declare_parameter('switch_stop_s', 0.2)   # 모드 전환 순간 정지 시간
        self.switch_stop_s = float(self.get_parameter('switch_stop_s').value)

        self.last_toggle_state = 0
        self.hold_until = 0.0

        self.last_manual = twist_zero()
        self.last_manual_t = 0.0

        self.last_auto = twist_zero()
        self.last_auto_t = 0.0

        self.pub_cmd = self.create_publisher(Twist, '/cmd_out', 10)
        self.pub_mode = self.create_publisher(UInt8, '/drive_mode', 10)

        self.sub_joy = self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.sub_manual = self.create_subscription(Twist, '/cmd_manual', self.on_manual, 10)
        self.sub_auto = self.create_subscription(Twist, '/cmd_auto', self.on_auto, 10)

        self.timer = self.create_timer(0.05, self.on_timer)  # 20Hz

        self.publish_mode()

    def publish_mode(self):
        m = UInt8()
        m.data = int(self.mode)
        self.pub_mode.publish(m)

    def on_joy(self, msg: Joy):
        # STOP: 즉시 정지 + MANUAL
        if 0 <= self.stop_button < len(msg.buttons) and msg.buttons[self.stop_button] == 1:
            if self.mode != MODE_MANUAL:
                self.mode = MODE_MANUAL
                self.publish_mode()
            self.pub_cmd.publish(twist_zero())
            return

        # TOGGLE on rising edge
        cur = 1 if (0 <= self.toggle_button < len(msg.buttons) and msg.buttons[self.toggle_button] == 1) else 0
        if cur == 1 and self.last_toggle_state == 0:
            # 1) 전환 순간 즉시 정지 + hold 시작
            self.pub_cmd.publish(twist_zero())
            self.hold_until = time.time() + self.switch_stop_s

            # 2) 모드 토글
            self.mode = MODE_AUTO if self.mode == MODE_MANUAL else MODE_MANUAL
            self.publish_mode()

        self.last_toggle_state = cur

    def on_manual(self, msg: Twist):
        self.last_manual = msg
        self.last_manual_t = time.time()

    def on_auto(self, msg: Twist):
        self.last_auto = msg
        self.last_auto_t = time.time()

    def on_timer(self):
        now = time.time()

        # (C) switching hold: 일정 시간 동안 무조건 정지 출력
        if now < self.hold_until:
            self.pub_cmd.publish(twist_zero())
            return

        if self.mode == MODE_AUTO:
            # auto cmd stale -> safety stop + MANUAL로 강등
            if now - self.last_auto_t > self.auto_timeout:
                self.pub_cmd.publish(twist_zero())
                self.mode = MODE_MANUAL
                self.publish_mode()
                return
            self.pub_cmd.publish(self.last_auto)
            return

        # MANUAL
        if now - self.last_manual_t > self.manual_timeout:
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
