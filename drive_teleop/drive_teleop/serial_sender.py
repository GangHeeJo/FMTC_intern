#!/usr/bin/env python3
import time
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class SerialSender(Node):
    def __init__(self):
        super().__init__('serial_sender')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        # Twist -> TH/ST mapping
        self.declare_parameter('th_threshold', 0.3)     # linear.x threshold
        self.declare_parameter('st_deadzone', 0.25)     # angular.z deadzone
        self.declare_parameter('st_steps', 2)           # -2..2
        self.declare_parameter('timeout_s', 0.3)        # cmd_out 끊기면 정지

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)

        self.th_th = float(self.get_parameter('th_threshold').value)
        self.st_dz = float(self.get_parameter('st_deadzone').value)
        self.st_steps = int(self.get_parameter('st_steps').value)
        self.timeout = float(self.get_parameter('timeout_s').value)

        self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
        time.sleep(0.5)
        self.get_logger().info(f"Opened serial: {self.port} @ {self.baud}")

        self.last_cmd_t = time.time()
        self.last_sent = None

        self.sub = self.create_subscription(Twist, '/cmd_out', self.on_cmd, 10)
        self.timer = self.create_timer(0.05, self.on_timer)  # watchdog

    def send(self, th: int, st: int):
        line = f"TH:{th} ST:{st}\n"
        if line != self.last_sent:
            self.ser.write(line.encode('ascii'))
            self.last_sent = line

    def on_cmd(self, msg: Twist):
        self.last_cmd_t = time.time()

        lin = msg.linear.x
        ang = msg.angular.z

        # TH: -1/0/+1
        if lin > self.th_th:
            th = 1
        elif lin < -self.th_th:
            th = -1
        else:
            th = 0

        # ST: -steps..+steps
        if abs(ang) < self.st_dz:
            st = 0
        else:
            st = int(round(ang * self.st_steps))
            st = clamp(st, -self.st_steps, self.st_steps)

        self.send(th, st)

    def on_timer(self):
        if time.time() - self.last_cmd_t > self.timeout:
            self.send(0, 0)

    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = SerialSender()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
