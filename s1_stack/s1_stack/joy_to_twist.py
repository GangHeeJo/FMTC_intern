#!/usr/bin/env python3
# joy_to_twist: 조이스틱 축(/joy) 값을 운전 명령(Twist)으로 바꿔 /cmd_manual로 내보냄.
# cmd_mux가 MANUAL 모드일 때 이 토픽을 그대로 써서 수동 운전함.
# 전후진은 비례 제어가 아니라 "임계값만 넘으면 최고 속도"(on-off) 방식이라 스틱을 살짝만 밀어도 확 나감.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def clamp(x, lo, hi):
    """x를 [lo, hi] 범위로 자름"""
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

        self.axis_steer = int(self.get_parameter('axis_steer').value)
        self.axis_drive = int(self.get_parameter('axis_drive').value)
        self.steer_deadzone = float(self.get_parameter('steer_deadzone').value)
        self.drive_threshold = float(self.get_parameter('drive_threshold').value)
        self.stn_scale = float(self.get_parameter('stn_scale').value)
        self.invert_steer = bool(self.get_parameter('invert_steer').value)
        self.invert_drive = bool(self.get_parameter('invert_drive').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

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
        """조이스틱 입력마다 호출. 최신 조향/속도 값을 저장만 해두고, 실제 발행은 on_timer가 주기적으로 함"""
        steer = msg.axes[self.axis_steer] if self.axis_steer < len(msg.axes) else 0.0
        drive = msg.axes[self.axis_drive] if self.axis_drive < len(msg.axes) else 0.0

        if self.invert_steer:
            steer = -steer
        if self.invert_drive:
            drive = -drive

        # steering: proportional — 스틱을 기울인 만큼 조향도 비례해서 커짐 (deadzone 안쪽은 0)
        if abs(steer) < self.steer_deadzone:
            steer = 0.0
        stn = steer * self.stn_scale
        stn = clamp(stn, -self.stn_scale, self.stn_scale)

        # drive: full-scale manual — 비례가 아니라 on-off. drive_threshold만 넘으면 바로 최고 속도(±1.0)
        if drive > self.drive_threshold:
            lin = 1.0
        elif drive < -self.drive_threshold:
            lin = -1.0
        else:
            lin = 0.0

        self.latest_lin = float(lin)
        self.latest_stn = float(stn)
        self.have_joy = True

    def on_timer(self):
        """publish_rate_hz(기본 20Hz)로 최신 조이스틱 값을 /cmd_manual로 발행. 조이스틱이 한 번도
        안 잡히면(have_joy=False) 아예 발행하지 않음 — cmd_mux 쪽에서 timeout으로 정지 처리됨"""
        if not self.have_joy:
            return

        cmd = Twist()
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
