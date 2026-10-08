#!/usr/bin/env python3
"""Bounded command transport; dry-run never opens a serial device."""
import math
import time
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String, Bool


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class SerialSender(Node):
    RX_LINE_MAX = 256
    RX_BYTES_PER_POLL = 256

    def __init__(self):
        super().__init__('serial_sender')
        defaults = {
            'port': '/dev/ttyACM0', 'baud': 115200,
            'th_threshold': 0.05, 'th_scale': 1000,
            'max_throttle': 1000, 'stn_max': 1000, 'stn_deadzone': 50,
            'timeout_s': 0.3, 'calib_cmd': 'CAL:START',
            # Retained for old YAML; elapsed time NEVER establishes readiness.
            'calib_lock_s': 6.0,
            'require_calibration_ack': True, 'dry_run': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        self.port, self.baud = str(param('port')), int(param('baud'))
        self.th_th, self.th_scale = float(param('th_threshold')), int(param('th_scale'))
        self.max_throttle = int(param('max_throttle'))
        self.stn_max, self.stn_deadzone = int(param('stn_max')), int(param('stn_deadzone'))
        self.timeout, self.calib_cmd = float(param('timeout_s')), str(param('calib_cmd'))
        self.require_ack, self.dry_run = bool(param('require_calibration_ack')), bool(param('dry_run'))
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('timeout_s must be finite and positive')
        if not math.isfinite(self.th_th) or not 0 <= self.th_th <= 1:
            raise ValueError('th_threshold must be within 0..1')
        if not 1 <= self.th_scale <= 1000 or not 0 <= self.max_throttle <= 1000:
            raise ValueError('th_scale must be 1..1000 and max_throttle 0..1000')
        if not 1 <= self.stn_max <= 1000 or not 0 <= self.stn_deadzone <= self.stn_max:
            raise ValueError('invalid steering range/deadzone')
        if self.calib_cmd != 'CAL:START':
            raise ValueError('only CAL:START is supported')

        self.ser = None
        self.last_cmd_t = None
        self.last_sent = None
        self.safety_stop_sent = False
        self.ready = self.dry_run or not self.require_ack
        self.awaiting_neutral = not self.dry_run
        self.awaiting_calibration_start = False
        self.calibration_in_progress = False
        self.io_failed = False
        self.rx_line = bytearray()
        self.rx_overflow = False
        self.last_status_t = None
        self.pub_tx = self.create_publisher(String, '/serial_tx', 10)
        self.pub_rx = self.create_publisher(String, '/serial_rx', 10)
        self.pub_ready = self.create_publisher(Bool, '/serial_ready', 10)
        if self.dry_run:
            self.get_logger().info('DRY RUN: no device opened; /serial_ready and /serial_tx are simulated')
        else:
            # Do not sleep or clear buffers: boot calibration replies are needed.
            self.ser = serial.Serial(self.port, self.baud, timeout=0, write_timeout=0.1)
            self.get_logger().info(f'Opened {self.port} @ {self.baud}; calibration acknowledgement required={self.require_ack}')
        self.sub_cmd = self.create_subscription(Twist, '/cmd_out', self.on_cmd, 10)
        self.sub_calib = self.create_subscription(Empty, '/calib/start', self.on_calib, 10)
        self.timer = self.create_timer(0.05, self.on_timer)

    def transport_failed(self, error):
        self.io_failed = True
        self.ready = False
        self.last_cmd_t = None
        self.get_logger().error(f'Serial unavailable; commands blocked: {error}')

    def send_line(self, line):
        if self.io_failed:
            return False
        line = line.rstrip('\r\n')
        if '\r' in line or '\n' in line:
            raise ValueError('send_line accepts exactly one line')
        wire = line + '\n'
        data = wire.encode('ascii')
        try:
            if not self.dry_run and self.ser.write(data) != len(data):
                raise OSError('partial serial write')
        except (OSError, serial.SerialException) as error:
            self.transport_failed(error)
            return False
        self.last_sent = wire
        msg = String()
        msg.data = wire
        self.pub_tx.publish(msg)
        return True

    def send_th_stn(self, th, stn):
        limit = min(self.th_scale, self.max_throttle)
        th = int(clamp(th, -limit, limit))
        stn = int(clamp(stn, -self.stn_max, self.stn_max))
        return self.send_line(f'TH:{th} STN:{stn}')

    def stop_once(self):
        if not self.safety_stop_sent:
            self.send_th_stn(0, 0)
            self.safety_stop_sent = True

    def handle_status(self, line):
        msg = String()
        msg.data = line
        self.pub_rx.publish(msg)
        if line in ('BOOT', 'CALIB:START', 'STATUS:NOT_READY') or line.startswith('CALIB:FAILED'):
            self.ready = False
            self.awaiting_neutral = True
            self.last_cmd_t = None
            if line == 'CALIB:START':
                self.awaiting_calibration_start = False
                self.calibration_in_progress = True
            elif line.startswith('CALIB:FAILED'):
                self.calibration_in_progress = False
            self.stop_once()
            self.get_logger().warn(f'Firmware not ready: {line}')
        elif line in ('CALIB:DONE', 'STATUS:READY'):
            if self.awaiting_calibration_start or (line == 'STATUS:READY' and self.calibration_in_progress):
                return
            was_ready = self.ready
            self.ready = True
            self.calibration_in_progress = False
            # Never replay commands received before completion.
            if not was_ready:
                self.last_cmd_t = None
                self.awaiting_neutral = True
                self.get_logger().info('Firmware calibrated; waiting for a neutral throttle AND steering command')
            self.get_logger().info(f'Firmware ready: {line}')

    def poll_serial(self):
        if self.dry_run or self.io_failed:
            return
        try:
            data = self.ser.read(min(self.RX_BYTES_PER_POLL, self.ser.in_waiting))
        except (OSError, serial.SerialException) as error:
            self.transport_failed(error)
            return
        for value in data:
            if value == 10:
                if not self.rx_overflow:
                    self.handle_status(self.rx_line.decode('ascii', errors='replace').rstrip('\r'))
                self.rx_line.clear()
                self.rx_overflow = False
            elif not self.rx_overflow:
                if len(self.rx_line) < self.RX_LINE_MAX:
                    self.rx_line.append(value)
                else:
                    self.rx_line.clear()
                    self.rx_overflow = True

    def on_calib(self, _msg):
        self.poll_serial()
        if self.awaiting_calibration_start or self.calibration_in_progress:
            self.get_logger().warn('Calibration already requested/in progress; duplicate request ignored')
            return
        self.ready = False
        self.awaiting_neutral = True
        self.awaiting_calibration_start = True
        self.calibration_in_progress = False
        self.last_cmd_t = None
        self.stop_once()
        self.send_line(self.calib_cmd)
        self.get_logger().info('Calibration requested; waiting for firmware completion (no timed unlock)')

    def on_cmd(self, msg):
        self.poll_serial()
        lin, ang = float(msg.linear.x), float(msg.angular.z)
        if not math.isfinite(lin) or not math.isfinite(ang):
            self.last_cmd_t = None
            self.stop_once()
            return
        if not self.ready or self.io_failed:
            self.last_cmd_t = None
            self.stop_once()
            return
        lin = clamp(lin, -1.0, 1.0)
        th = 0 if abs(lin) < self.th_th else int(round(lin * self.th_scale))
        stn = int(round(clamp(ang, -self.stn_max, self.stn_max)))
        if abs(stn) < self.stn_deadzone:
            stn = 0
        if self.awaiting_neutral:
            self.last_cmd_t = None
            self.stop_once()
            if th == 0 and stn == 0:
                self.awaiting_neutral = False
                self.get_logger().info('Neutral command received; subsequent commands enabled')
            return
        self.last_cmd_t = time.monotonic()
        self.safety_stop_sent = False
        self.send_th_stn(th, stn)

    def on_timer(self):
        self.poll_serial()
        now = time.monotonic()
        if self.last_cmd_t is not None and now - self.last_cmd_t >= self.timeout:
            self.stop_once()
            self.last_cmd_t = None
        if not self.dry_run and not self.ready and not self.io_failed:
            if self.last_status_t is None or now - self.last_status_t >= 1.0:
                # Patched firmware answers STATUS. Old firmware may emit DONE;
                # UNKNOWN:STATUS never unlocks the gate.
                self.send_line('STATUS')
                self.last_status_t = now
        msg = Bool()
        msg.data = self.ready and not self.io_failed
        self.pub_ready.publish(msg)

    def destroy_node(self):
        try:
            self.send_th_stn(0, 0)
        finally:
            try:
                if self.ser is not None:
                    self.ser.close()
            finally:
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
