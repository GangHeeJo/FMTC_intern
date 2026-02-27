#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class JoyToTwist(Node):
    def __init__(self):
        super().__init__('joy_to_twist')

        self.declare_parameter('axis_throttle', 1)   # 위/아래
        self.declare_parameter('axis_steer', 0)      # 좌/우
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('scale_linear', 1.0)  # -1..1
        self.declare_parameter('scale_angular', 1.0) # -1..1

        self.axis_throttle = int(self.get_parameter('axis_throttle').value)
        self.axis_steer = int(self.get_parameter('axis_steer').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.scale_linear = float(self.get_parameter('scale_linear').value)
        self.scale_angular = float(self.get_parameter('scale_angular').value)

        self.pub = self.create_publisher(Twist, '/cmd_manual', 10)
        self.sub = self.create_subscription(Joy, '/joy', self.on_joy, 10)

    def on_joy(self, msg: Joy):
        thr = msg.axes[self.axis_throttle] if self.axis_throttle < len(msg.axes) else 0.0
        steer = msg.axes[self.axis_steer] if self.axis_steer < len(msg.axes) else 0.0

        if abs(thr) < self.deadzone: thr = 0.0
        if abs(steer) < self.deadzone: steer = 0.0

        # 너 컨트롤러 기준: 아래(+)=전진, 위(-)=후진이라 했었음
        # Twist.linear.x는 "전진(+)"가 보통이라 아래(+)를 그대로 전진으로 둠.
        cmd = Twist()
        cmd.linear.x = clamp(thr * self.scale_linear, -1.0, 1.0)
        cmd.angular.z = clamp((-steer) * self.scale_angular, -1.0, 1.0)  # 좌/우 방향 보정(-)
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
