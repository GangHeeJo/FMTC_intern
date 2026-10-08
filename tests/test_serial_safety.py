"""Actual sender definitions + actual .ino host build; no ROS/USB/physical model."""
import ast
import copy
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(copy.deepcopy(msg))


class FakeSerial:
    def __init__(self):
        self.rx = bytearray()
        self.writes = []
        self.closed = False
        self.partial_write = False
        self.read_error = False

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, size):
        if self.read_error:
            raise OSError('unplugged')
        data = bytes(self.rx[:size])
        del self.rx[:size]
        return data

    def write(self, data):
        self.writes.append(data)
        return 1 if self.partial_write else len(data)

    def close(self):
        self.closed = True


def command(lin=0.5, ang=100.0):
    return SimpleNamespace(linear=SimpleNamespace(x=lin), angular=SimpleNamespace(z=ang))


def load_sender(overrides=None):
    settings = overrides or {}
    fake_serial = FakeSerial()
    opened = []
    clock = SimpleNamespace(now=10.0)
    clock.monotonic = lambda: clock.now

    class Node:
        def __init__(self, name):
            self.params = {}

        def declare_parameter(self, name, default):
            self.params[name] = settings.get(name, default)

        def get_parameter(self, name):
            return SimpleNamespace(value=self.params[name])

        def create_publisher(self, *args):
            return Publisher()

        def create_subscription(self, *args):
            pass

        def create_timer(self, *args):
            pass

        def get_logger(self):
            return SimpleNamespace(info=lambda *a: None, warn=lambda *a: None, error=lambda *a: None)

        def destroy_node(self):
            pass

    def factory(*args, **kwargs):
        opened.append((args, kwargs))
        return fake_serial

    path = ROOT / 's1_stack/s1_stack/serial_sender.py'
    tree = ast.parse(path.read_text())
    tree.body = [item for item in tree.body if not isinstance(item, (ast.Import, ast.ImportFrom))]
    scope = dict(__name__='serial_test', Node=Node, Twist=SimpleNamespace,
                 Empty=SimpleNamespace, String=SimpleNamespace, Bool=SimpleNamespace,
                 time=clock, math=math,
                 serial=SimpleNamespace(Serial=factory, SerialException=OSError))
    exec(compile(tree, str(path), 'exec'), scope)
    return scope['SerialSender'](), fake_serial, clock, opened


class SerialSafetyTests(unittest.TestCase):
    def test_dry_run_never_opens_serial_caps_throttle_and_publishes(self):
        node, port, clock, opened = load_sender({'dry_run': True, 'max_throttle': 200})
        node.on_cmd(command(1.0, 10000))
        self.assertEqual(opened, [])
        self.assertEqual(port.writes, [])
        self.assertEqual(node.pub_tx.messages[-1].data, 'TH:200 STN:1000\n')
        node.on_cmd(command(-1, -10000))
        self.assertEqual(node.pub_tx.messages[-1].data, 'TH:-200 STN:-1000\n')
        node.on_timer()
        self.assertTrue(node.pub_ready.messages[-1].data)

    def test_requires_real_ack_not_elapsed_time_or_unknown_status(self):
        node, port, clock, opened = load_sender()
        node.on_cmd(command())
        self.assertEqual(port.writes, [b'TH:0 STN:0\n'])
        clock.now += 60
        port.rx.extend(b'UNKNOWN:STATUS\n')
        node.on_timer()
        node.on_cmd(command())
        self.assertFalse(node.ready)
        self.assertNotIn(b'TH:500 STN:100\n', port.writes)
        port.rx.extend(b'STATUS:READY\n')
        node.on_timer()
        self.assertTrue(node.ready)
        self.assertIsNone(node.last_cmd_t)
        node.on_cmd(command(0, 0))
        node.on_cmd(command())
        self.assertEqual(port.writes[-1], b'TH:500 STN:100\n')

    def test_legacy_boot_ack_split_reads_and_failure(self):
        node, port, clock, _ = load_sender()
        port.rx.extend(b'BOOT\nCALIB:START\r\nCALIB:DO')
        node.on_timer()
        self.assertFalse(node.ready)
        port.rx.extend(b'NE\r\n')
        node.on_cmd(command(0, 0))
        node.on_cmd(command())
        self.assertTrue(node.ready)
        self.assertEqual(port.writes[-1], b'TH:500 STN:100\n')
        port.rx.extend(b'CALIB:FAILED center timeout\n')
        node.on_cmd(command())
        self.assertFalse(node.ready)
        self.assertEqual(port.writes[-1], b'TH:0 STN:0\n')
        self.assertEqual(node.pub_rx.messages[-1].data, 'CALIB:FAILED center timeout')

    def test_timeout_is_one_stop_then_silence(self):
        node, port, clock, _ = load_sender({'dry_run': True})
        node.on_cmd(command())
        clock.now += 0.29
        node.on_timer()
        self.assertEqual(len(node.pub_tx.messages), 1)
        clock.now += 0.02
        node.on_timer()
        self.assertEqual(node.pub_tx.messages[-1].data, 'TH:0 STN:0\n')
        for _ in range(60):
            clock.now += 0.1
            node.on_timer()
        self.assertEqual(len(node.pub_tx.messages), 2)
        node.on_cmd(command())
        self.assertEqual(node.pub_tx.messages[-1].data, 'TH:500 STN:100\n')

    def test_invalid_inputs_stop_without_heartbeat(self):
        for lin, ang in ((float('nan'), 0), (0.5, float('inf')), (-float('inf'), 0)):
            node, port, clock, _ = load_sender({'dry_run': True})
            node.on_cmd(command())
            node.on_cmd(command(lin, ang))
            node.on_cmd(command(lin, ang))
            self.assertEqual([m.data for m in node.pub_tx.messages], ['TH:500 STN:100\n', 'TH:0 STN:0\n'])

    def test_calibration_needs_start_then_done_and_never_replays(self):
        node, port, clock, _ = load_sender()
        port.rx.extend(b'CALIB:DONE\n')
        node.on_cmd(command())
        node.on_calib(None)
        self.assertEqual(port.writes[-2:], [b'TH:0 STN:0\n', b'CAL:START\n'])
        port.rx.extend(b'STATUS:READY\nCALIB:DONE\n')
        node.on_timer()
        self.assertFalse(node.ready)
        port.rx.extend(b'CALIB:START\nSTATUS:READY\n')
        node.on_timer()
        self.assertFalse(node.ready)
        clock.now += 100
        node.on_cmd(command())
        self.assertFalse(node.ready)
        port.rx.extend(b'CALIB:DONE\n')
        node.on_timer()
        self.assertTrue(node.ready)
        self.assertIsNone(node.last_cmd_t)
        self.assertNotEqual(port.writes[-1], b'TH:500 STN:100\n')

    def test_overlong_uart_line_bounded_and_tail_not_ack(self):
        node, port, clock, _ = load_sender()
        port.rx.extend(b'X' * 800 + b'CALIB:DONE\n')
        while port.in_waiting:
            node.on_timer()
            self.assertLessEqual(len(node.rx_line), node.RX_LINE_MAX)
            self.assertFalse(node.ready)
        port.rx.extend(b'CALIB:DONE\n')
        node.on_timer()
        self.assertTrue(node.ready)

    def test_repeated_ready_ack_does_not_cancel_timeout(self):
        node, port, clock, _ = load_sender()
        port.rx.extend(b'CALIB:DONE\n')
        node.on_cmd(command(0, 0))
        node.on_cmd(command())
        clock.now += 0.2
        port.rx.extend(b'STATUS:READY\n')
        node.on_timer()
        clock.now += 0.11
        node.on_timer()
        self.assertEqual(port.writes[-1], b'TH:0 STN:0\n')

    def test_explicit_legacy_opt_out_still_waits_after_recalibration(self):
        node, port, clock, _ = load_sender({'require_calibration_ack': False})
        node.on_cmd(command(0, 0))
        node.on_cmd(command())
        self.assertEqual(port.writes[-1], b'TH:500 STN:100\n')
        node.on_calib(None)
        clock.now += 60
        node.on_cmd(command())
        self.assertFalse(node.ready)
        self.assertEqual(port.writes[-1], b'CAL:START\n')

    def test_ready_requires_neutral_before_held_drive_and_after_reboot(self):
        node, port, clock, _ = load_sender()
        port.rx.extend(b'CALIB:DONE\n')
        node.on_cmd(command())
        self.assertTrue(node.ready)
        self.assertTrue(node.awaiting_neutral)
        self.assertEqual(port.writes, [b'TH:0 STN:0\n'])
        node.on_cmd(command())
        node.on_cmd(command(0, 500))
        self.assertTrue(node.awaiting_neutral)
        self.assertEqual(len(port.writes), 1)
        node.on_cmd(command(0, 0))
        self.assertFalse(node.awaiting_neutral)
        node.on_cmd(command())
        self.assertEqual(port.writes[-1], b'TH:500 STN:100\n')
        port.rx.extend(b'BOOT\nCALIB:START\nCALIB:DONE\n')
        node.on_cmd(command())
        self.assertTrue(node.awaiting_neutral)
        self.assertEqual(port.writes[-1], b'TH:0 STN:0\n')

    def test_duplicate_calibration_requests_do_not_wait_for_discarded_command(self):
        node, port, clock, _ = load_sender()
        port.rx.extend(b'CALIB:DONE\n')
        node.on_cmd(command(0, 0))
        node.on_calib(None)
        self.assertTrue(node.awaiting_calibration_start)
        node.on_calib(None)
        self.assertEqual(port.writes.count(b'CAL:START\n'), 1)
        port.rx.extend(b'CALIB:START\n')
        node.on_calib(None)
        self.assertTrue(node.calibration_in_progress)
        self.assertFalse(node.awaiting_calibration_start)
        self.assertEqual(port.writes.count(b'CAL:START\n'), 1)
        port.rx.extend(b'CALIB:DONE\n')
        node.on_timer()
        self.assertTrue(node.ready)
        self.assertTrue(node.awaiting_neutral)
        node.on_cmd(command())
        self.assertNotEqual(port.writes[-1], b'TH:500 STN:100\n')
        node.on_cmd(command(0, 0))
        node.on_cmd(command())
        self.assertEqual(port.writes[-1], b'TH:500 STN:100\n')

    def test_newline_always_sent_once_and_shutdown_stop(self):
        node, port, _, _ = load_sender()
        node.send_line('STATUS\n')
        self.assertEqual(port.writes, [b'STATUS\n'])
        with self.assertRaises(ValueError):
            node.send_line('TH:500 STN:0\nSTATUS')
        node.destroy_node()
        self.assertEqual(port.writes[-1], b'TH:0 STN:0\n')
        self.assertTrue(port.closed)

    def test_io_failure_latches_block(self):
        for failure in ('read_error', 'partial_write'):
            node, port, clock, _ = load_sender()
            port.rx.extend(b'CALIB:DONE\n')
            node.on_timer()
            setattr(port, failure, True)
            node.on_cmd(command())
            self.assertTrue(node.io_failed)
            self.assertFalse(node.ready)
            writes = len(port.writes)
            node.on_cmd(command())
            self.assertEqual(len(port.writes), writes)

    def test_parameter_validation(self):
        for config in ({'max_throttle': -1}, {'max_throttle': 1001}, {'th_scale': 2000},
                       {'timeout_s': 0}, {'timeout_s': float('nan')}, {'stn_max': 2000}):
            with self.subTest(config=config), self.assertRaises(ValueError):
                load_sender(config)


class FirmwareHostTests(unittest.TestCase):
    def test_actual_firmware_scenarios(self):
        compiler = shutil.which('clang++') or shutil.which('g++')
        if compiler is None:
            self.skipTest('host C++ compiler unavailable; firmware scenarios not run')
        with tempfile.TemporaryDirectory(prefix='s1-firmware-test-') as output:
            binary = str(Path(output) / 'test_s1')
            result = subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra',
                                     '-I', str(ROOT / 'tests/firmware'),
                                     str(ROOT / 'tests/firmware/test_s1.cpp'), '-o', binary],
                                    capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([binary], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
