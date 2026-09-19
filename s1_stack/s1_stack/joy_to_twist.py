#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class JoyToTwist(Node):
    def __init__(self):
        super().__init__('joy_to_twist')

        # ---- axes mapping ----
        self.declare_parameter('axis_steer', 0)          # left stick L/R
        self.declare_parameter('axis_drive', 3)          # right stick U/D

        # deadzones / thresholds
        self.declare_parameter('steer_deadzone', 0.15)
        self.declare_parameter('drive_threshold', 0.10)

        # scaling
        self.declare_parameter('stn_scale', 1000.0)
        self.declare_parameter('invert_steer', False)
        self.declare_parameter('invert_drive', False)

        # periodic publish
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('joy_timeout_s', 0.5)

        self.axis_steer = int(self.get_parameter('axis_steer').value)
        self.axis_drive = int(self.get_parameter('axis_drive').value)
        self.steer_deadzone = float(self.get_parameter('steer_deadzone').value)
        self.drive_threshold = float(self.get_parameter('drive_threshold').value)
        self.stn_scale = float(self.get_parameter('stn_scale').value)
        self.invert_steer = bool(self.get_parameter('invert_steer').value)
        self.invert_drive = bool(self.get_parameter('invert_drive').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.joy_timeout = float(self.get_parameter('joy_timeout_s').value)
        self.last_joy_t = None

        self.pub = self.create_publisher(Twist, '/cmd_manual', 10)
        self.sub = self.create_subscription(Joy, '/joy', self.on_joy, 10)

        self.latest_lin = 0.0
        self.latest_stn = 0.0
        self.have_joy = False

        timer_period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.on_timer)

        self.get_logger().info(
            f"joy_to_twist started: axis_steer={self.axis_steer}, "
            f"axis_drive={self.axis_drive}, publish_rate_hz={self.publish_rate_hz}"
        )

    def on_joy(self, msg: Joy):
        steer = msg.axes[self.axis_steer] if self.axis_steer < len(msg.axes) else 0.0
        drive = msg.axes[self.axis_drive] if self.axis_drive < len(msg.axes) else 0.0

        if self.invert_steer:
            steer = -steer
        if self.invert_drive:
            drive = -drive

        # steering: proportional
        if abs(steer) < self.steer_deadzone:
            steer = 0.0
        stn = steer * self.stn_scale
        stn = clamp(stn, -self.stn_scale, self.stn_scale)

        # drive: full-scale manual
        if drive > self.drive_threshold:
            lin = 1.0
        elif drive < -self.drive_threshold:
            lin = -1.0
        else:
            lin = 0.0

        self.latest_lin = float(lin)
        self.latest_stn = float(stn)
        self.have_joy = True
        self.last_joy_t = time.monotonic()

    def on_timer(self):
        if not self.have_joy:
            return

        cmd = Twist()
        if time.monotonic() - self.last_joy_t < self.joy_timeout:
            cmd.linear.x = self.latest_lin
            cmd.angular.z = self.latest_stn
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = JoyToTwist()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
