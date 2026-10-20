#!/usr/bin/env python3
# serial_sender: 파이프라인의 맨 끝. cmd_mux가 내보낸 최종 명령(/cmd_out, Twist)을 받아서
# 아두이노가 이해하는 텍스트 한 줄("TH:<-1000..1000> STN:<-1000..1000>\n")로 바꿔 시리얼로 전송.
# rosserial 같은 ROS 메시지 프로토콜이 아니라 그냥 우리가 정한 문자열 프로토콜(S1.ino가 파싱함).
import time
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from rcl_interfaces.msg import SetParametersResult


def clamp(x, lo, hi):
    """x를 [lo, hi] 범위로 자름"""
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
        self.declare_parameter('timeout_s', 0.3)  # 선언만 돼있고 아래 코드에서 실제로는 안 씀(참고용)
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
        # 속도(th_scale)는 노드를 다시 켜지 않고 ros2 param set으로 바로 바꿀 수 있게 함
        self.add_on_set_parameters_callback(self.on_params)

        # USB가 순간적으로 끊겨도(허브 EMI 등) 노드가 죽지 않고 정지 상태로 대기하며 재연결한다
        self.reconnect_period_s = 1.0
        self.boot_wait_s = 2.0    # 포트를 열면 아두이노가 리셋되므로 그동안은 명령을 보내지 않음
        self.ser = None
        self.ready_at = 0.0
        self.last_try_t = 0.0

        self.last_cmd_t = time.time()
        self.last_sent = None
        self.calib_until = 0.0

        self.try_open()

        self.sub_cmd = self.create_subscription(Twist, '/cmd_out', self.on_cmd, 10)
        self.sub_calib = self.create_subscription(Empty, '/calib/start', self.on_calib, 10)
        self.timer = self.create_timer(0.05, self.on_timer)

    def on_params(self, params):
        for p in params:
            if p.name == 'th_scale':
                v = int(p.value)
                if not 0 <= v <= 1000:
                    return SetParametersResult(successful=False, reason='th_scale은 0~1000')
                self.th_scale = v
                self.get_logger().info(f"th_scale = {v}")
        return SetParametersResult(successful=True)

    def try_open(self):
        """시리얼 포트 열기 시도. 실패해도 예외를 던지지 않고 False만 반환 — on_timer가 주기적으로 재시도함"""
        self.last_try_t = time.time()
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.get_logger().warn(f"Serial open failed ({e}); retrying in {self.reconnect_period_s}s",
                                   throttle_duration_sec=5.0)
            return False
        # 포트를 여는 순간 아두이노가 리셋되기 때문에, boot_wait_s가 지나기 전엔 명령을 보내지 않음
        self.ready_at = time.time() + self.boot_wait_s
        self.get_logger().info(f"Opened serial: {self.port} @ {self.baud}")
        return True

    def drop_serial(self, reason):
        """쓰기 중 오류가 나면(USB 뽑힘 등) 포트를 닫고 self.ser=None으로 — 이후 on_timer가 재연결 시도"""
        self.get_logger().error(f"Serial lost ({reason}); stopped, will reconnect")
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def send_line(self, line: str):
        if not line.endswith('\n'):
            line += '\n'

        # 연결이 없거나 아두이노 부팅 중이면 보내지 않음 (정지 유지)
        if self.ser is None or time.time() < self.ready_at:
            return
        try:
            self.ser.write(line.encode('ascii'))
            self.last_sent = line
        except (serial.SerialException, OSError) as e:
            self.drop_serial(e)

    def send_th_stn(self, th: int, stn: int):
        """TH(전후진)/STN(조향)을 안전 범위로 자른 뒤 "TH:.. STN:.."로 한 줄 전송"""
        th = int(clamp(th, -self.th_scale, self.th_scale))
        stn = int(clamp(stn, -self.stn_max, self.stn_max))
        self.send_line(f"TH:{th} STN:{stn}")

    def on_calib(self, _msg: Empty):
        """조향 센터값 등 캘리브레이션 시작 신호. 아두이노에 CAL 명령을 보내고, calib_lock_s초 동안은
        on_cmd가 들어와도 무조건 정지(0,0)만 보내서 캘리브레이션 중 차가 움직이지 않게 함"""
        self.send_th_stn(0, 0)
        self.calib_until = time.time() + self.calib_lock_s
        self.send_line(self.calib_cmd)
        self.get_logger().info(f"Sent {self.calib_cmd}, lock {self.calib_lock_s}s")

    def on_cmd(self, msg: Twist):
        """최종 명령(/cmd_out) 수신 시마다 호출. Twist(linear.x 속도, angular.z 조향)를
        아두이노용 정수 TH/STN 값으로 변환해서 즉시 전송"""
        now = time.time()
        self.last_cmd_t = now

        if now < self.calib_until:
            self.send_th_stn(0, 0)
            return

        lin = clamp(float(msg.linear.x), -1.0, 1.0)
        ang = float(msg.angular.z)

        # proportional throttle: -1~1 범위의 속도를 th_scale(기본 1000)배로 늘림. th_threshold보다
        # 작은 값(거의 0)은 그냥 0으로 처리해 미세한 떨림으로 차가 기어가는 것을 막음
        if abs(lin) < self.th_th:
            th = 0
        else:
            th = int(round(lin * self.th_scale))

        # steering 그대로 사용: angular.z가 이미 상위 노드(lane_trace 등)에서 -1000~1000 스케일로 와서
        # 추가 변환 없이 범위만 자르고, stn_deadzone보다 작은 조향은 0으로(센서 노이즈로 미세하게 틀어지는 것 방지)
        stn = int(round(ang))
        stn = int(clamp(stn, -self.stn_max, self.stn_max))
        if abs(stn) < self.stn_deadzone:
            stn = 0

        self.send_th_stn(th, stn)

    def on_timer(self):
        """20Hz. 연결이 끊긴 상태면 reconnect_period_s마다 재연결을 시도 (USB 흔들림/허브 재부팅 대비)"""
        if self.ser is None and time.time() - self.last_try_t >= self.reconnect_period_s:
            self.try_open()

    def destroy_node(self):
        try:
            if self.ser is not None:
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
