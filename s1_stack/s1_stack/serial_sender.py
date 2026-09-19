#!/usr/bin/env python3
import time
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class SerialSender(Node):
    def __init__(self):
        super().__init__('serial_sender')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        # throttle / steering conversion
        self.declare_parameter('th_threshold', 0.05)   # below this, treat as zero
        self.declare_parameter('th_scale', 1000)       # linear.x(-1~1) -> TH(-1000~1000)
        self.declare_parameter('stn_max', 1000)
        self.declare_parameter('stn_deadzone', 50)

        # timeout / calibration
        self.declare_parameter('timeout_s', 0.3)
        self.declare_parameter('calib_lock_s', 6.0)
        self.declare_parameter('calib_cmd', 'CAL:START')

        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)

        self.th_th = float(self.get_parameter('th_threshold').value)
        self.th_scale = int(self.get_parameter('th_scale').value)
        self.stn_max = int(self.get_parameter('stn_max').value)
        self.stn_deadzone = int(self.get_parameter('stn_deadzone').value)

        self.timeout = float(self.get_parameter('timeout_s').value)
        self.calib_lock_s = float(self.get_parameter('calib_lock_s').value)
        self.calib_cmd = str(self.get_parameter('calib_cmd').value)

        self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
        time.sleep(2.0)
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception:
            pass

        self.get_logger().info(f"Opened serial: {self.port} @ {self.baud}")

        self.last_cmd_t = time.time()
        self.last_sent = None
        self.calib_until = 0.0

        self.sub_cmd = self.create_subscription(Twist, '/cmd_out', self.on_cmd, 10)
        self.sub_calib = self.create_subscription(Empty, '/calib/start', self.on_calib, 10)
        self.timer = self.create_timer(0.05, self.on_timer)

    def send_line(self, line: str):
        if not line.endswith('\n'):
            line += '\n'

#        if line != self.last_sent:               3.py : #
            self.ser.write(line.encode('ascii'))
            self.last_sent = line

    def send_th_stn(self, th: int, stn: int):
        th = int(clamp(th, -self.th_scale, self.th_scale))
        stn = int(clamp(stn, -self.stn_max, self.stn_max))
        self.send_line(f"TH:{th} STN:{stn}")

    def on_calib(self, _msg: Empty):
        self.send_th_stn(0, 0)
        self.calib_until = time.time() + self.calib_lock_s
        self.send_line(self.calib_cmd)
        self.get_logger().info(f"Sent {self.calib_cmd}, lock {self.calib_lock_s}s")

    def on_cmd(self, msg: Twist):
        now = time.time()
        self.last_cmd_t = now

        if now < self.calib_until:
            self.send_th_stn(0, 0)
            return

        lin = clamp(float(msg.linear.x), -1.0, 1.0)
        ang = float(msg.angular.z)

        # proportional throttle
        if abs(lin) < self.th_th:
            th = 0
        else:
            th = int(round(lin * self.th_scale))

        # steering 그대로 사용
        stn = int(round(ang))
        stn = int(clamp(stn, -self.stn_max, self.stn_max))
        if abs(stn) < self.stn_deadzone:
            stn = 0

        self.send_th_stn(th, stn)

    def on_timer(self):
        
        pass

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
