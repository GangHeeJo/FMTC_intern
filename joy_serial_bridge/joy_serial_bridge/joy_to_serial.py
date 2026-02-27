#!/usr/bin/env python3
import time
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class JoyToSerial(Node):
    """
    Joystick(/joy) -> Serial ASCII commands to Arduino Mega.

    Protocol (one line):
      "TH:<0|1> ST:<-2..2>\n"
    - TH: throttle on/off (0 stop, 1 forward)
    - ST: steering step (-2 hard left .. 0 straight .. +2 hard right)
    """
    def __init__(self):
        super().__init__('joy_to_serial')

        # --- params ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        # joystick mapping (change later if needed)
        self.declare_parameter('axis_steer', 0)       # left stick L/R often 0
        self.declare_parameter('axis_throttle', 1)    # left stick U/D often 1
        self.declare_parameter('deadman_button', -1)  # -1 disables (use if you want)
        self.declare_parameter('stop_button', 1)      # B / Circle etc (example)

        # discretization
        self.declare_parameter('steer_steps', 2)      # output range: -2..+2
        self.declare_parameter('steer_deadzone', 0.2)
        self.declare_parameter('throttle_threshold', 0.3)  # stick forward > 0.3 => TH=1

        # safety
        self.declare_parameter('timeout_s', 0.3)      # if no joy msgs -> stop

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)

        self.axis_steer = int(self.get_parameter('axis_steer').value)
        self.axis_throttle = int(self.get_parameter('axis_throttle').value)
        self.deadman_button = int(self.get_parameter('deadman_button').value)
        self.stop_button = int(self.get_parameter('stop_button').value)

        self.steer_steps = int(self.get_parameter('steer_steps').value)
        self.steer_deadzone = float(self.get_parameter('steer_deadzone').value)
        self.throttle_threshold = float(self.get_parameter('throttle_threshold').value)

        self.timeout_s = float(self.get_parameter('timeout_s').value)

        self.ser = serial.Serial(self.port, self.baud, timeout=0.01)

	# --- DTR 제어로 안정적 리셋 ---
        self.ser.setDTR(False)
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.setDTR(True)

	# Mega 리셋 후 스케치 시작 대기
        time.sleep(2.0)

        self.get_logger().info(f"Opened serial: {self.port} @ {self.baud}")


        self.last_joy_time = time.time()
        self.last_sent = None

        self.sub = self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.timer = self.create_timer(0.05, self.on_timer)  # 20Hz watchdog

    def send_line(self, line: str):
        self.ser.write(line.encode('ascii'))

    def on_joy(self, msg: Joy):
        self.last_joy_time = time.time()

        # STOP button has priority
        if 0 <= self.stop_button < len(msg.buttons) and msg.buttons[self.stop_button] == 1:
            self.publish_cmd(0, 0)
            return

        # Optional deadman: must be held to move (구동만 막고 싶으면 여기 로직을 th만 0으로 바꾸면 됨)
        if self.deadman_button >= 0:
            if not (self.deadman_button < len(msg.buttons) and msg.buttons[self.deadman_button] == 1):
                self.publish_cmd(0, 0)
                return

        steer = msg.axes[self.axis_steer] if self.axis_steer < len(msg.axes) else 0.0
        thr = msg.axes[self.axis_throttle] if self.axis_throttle < len(msg.axes) else 0.0

        # throttle: 아래(+)=전진, 위(-)=후진
        if thr > self.throttle_threshold:
            th = 1
        elif thr < -self.throttle_threshold:
            th = -1
        else:
            th = 0

        # deadzone + discretize to -steps..+steps (부호는 너가 원한대로 뒤집어둠)
        if abs(steer) < self.steer_deadzone:
            st = 0
        else:
            st = -int(round(steer * self.steer_steps))
            st = clamp(st, -self.steer_steps, self.steer_steps)

        self.publish_cmd(th, st)
        self.get_logger().info(f"thr={thr:.2f} steer={steer:.2f} -> TH:{th} ST:{st}") 
           
    def publish_cmd(self, th: int, st: int):
        line = f"TH:{th} ST:{st}\n"
        self.get_logger().info(line.strip())
        if line != self.last_sent:
            self.send_line(line)
            self.last_sent = line

    def on_timer(self):
        # If joystick messages stop, force stop
        if time.time() - self.last_joy_time > self.timeout_s:
            self.publish_cmd(0, 0)

    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = JoyToSerial()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
