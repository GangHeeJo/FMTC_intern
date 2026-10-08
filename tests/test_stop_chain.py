"""Host integration of actual perception/control/sender callbacks to /serial_tx.

Fake detections and lane results feed real callbacks; messages are wired by hand.
No ROS/DDS, model inference, image accuracy, firmware or hardware is exercised.
The sender is dry-run, and every test verifies no serial factory was called.
"""
from types import SimpleNamespace
import unittest

import test_control_watchdogs as control
import test_perception_safety as perception
import test_serial_safety as serial_helpers


class StopChainTests(unittest.TestCase):
    def setUp(self):
        self.sender, self.port, self.clock, self.opened = serial_helpers.load_sender(
            {'dry_run': True, 'max_throttle': 200})
        self.lane_scope = perception.load_source('lane_trace/lane_trace/lane_trace.py')
        self.lane = self.lane_scope['LaneMaskingNode']()
        self.sign_scope = perception.load_source('sign_detect/sign_detect/sign_detect.py')
        self.sign = self.sign_scope['VisionPercept']()
        self.decision = control.load_node('decision_auto.py', 'MotionDecision', self.clock)
        self.mux = control.load_node('cmd_mux.py', 'CmdMux', self.clock)
        self.mux.require_joy_for_auto = True

        # Explicit neutral command also satisfies the post-readiness neutral gate.
        self.sender.on_cmd(serial_helpers.command(0.0, 0.0))
        self.mux.on_joy(SimpleNamespace(buttons=[1, 0, 0, 0]))
        self.sender.on_cmd(self.mux.pub_cmd.messages[-1])
        self.clock.now += self.mux.switch_stop_s + 0.01

    def tearDown(self):
        self.assertEqual(self.opened, [])
        self.assertEqual(self.port.writes, [])

    def frame(self, boxes=(), lane_steer=0.1, scan_ranges=(1.0, 2.0), evasion=False, image_stamp=None):
        self.clock.now += 0.1
        self.lane_scope['lane_detect'] = lambda image: (None, None, lane_steer)
        self.lane.image_callback('synthetic image')
        self.decision.lane_cb(self.lane.cmd_pub.messages[-1])

        self.sign.model.boxes = list(boxes)
        self.sign_scope['test_clock'].ros = self.clock.now
        self.sign_scope['test_clock'].wall = self.clock.now
        stamp = self.clock.now if image_stamp is None else image_stamp
        seconds = int(stamp)
        self.sign.image_callback(SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
            sec=seconds, nanosec=int((stamp-seconds)*1e9)))))
        self.decision.light_cb(self.sign.pub_light.messages[-1])
        self.decision.cross_cb(self.sign.pub_cross.messages[-1])

        if scan_ranges is not None:
            self.decision.scan_cb(SimpleNamespace(
                ranges=scan_ranges, range_min=0.1, range_max=10.0, angle_increment=0.01))
        if evasion:
            self.decision.obs_cb(serial_helpers.command(0.7, 1000.0))
        self.decision.publish_decision()

        # The commissioning auto mode requires a live operator joystick stream.
        self.mux.on_joy(SimpleNamespace(buttons=[0, 0, 0, 0]))
        self.mux.on_auto(self.decision.pub_auto.messages[-1])
        self.mux.on_timer()
        self.sender.on_cmd(self.mux.pub_cmd.messages[-1])
        return self.sender.pub_tx.messages[-1].data

    def throttle(self, line):
        self.assertTrue(line.endswith('\n'))
        throttle, steering = line.strip().split()
        self.assertTrue(throttle.startswith('TH:'))
        self.assertTrue(steering.startswith('STN:'))
        return int(throttle[3:])

    def red_stop(self):
        outputs = [self.frame([perception.box(2)]) for _ in range(3)]
        self.assertEqual([self.throttle(line) for line in outputs], [200, 200, 0])
        return outputs[-1]

    def test_red_detection_reaches_zero_serial_command_and_green_restores_cap(self):
        self.assertEqual(self.frame(), 'TH:200 STN:-250\n')
        self.assertEqual(self.red_stop(), 'TH:0 STN:-250\n')
        for _ in range(12):
            self.assertEqual(self.throttle(self.frame()), 0)
        self.assertEqual(self.throttle(self.frame([perception.box(1)])), 0)
        self.assertEqual(self.throttle(self.frame([perception.box(1)])), 0)
        self.assertEqual(self.frame([perception.box(1)]), 'TH:200 STN:-250\n')

    def test_lane_loss_reaches_zero_despite_fresh_image_and_live_evasion(self):
        self.assertEqual(self.throttle(self.frame(evasion=True)), 200)
        self.assertEqual(self.throttle(self.frame(lane_steer=None, evasion=True)), 0)
        self.assertEqual(self.throttle(self.frame(lane_steer=None, evasion=True)), 0)
        self.assertEqual(self.throttle(self.frame()), 200)

    def test_missing_stale_and_unusable_lidar_reach_zero_serial_command(self):
        self.assertEqual(self.throttle(self.frame(scan_ranges=None)), 0)
        self.assertEqual(self.throttle(self.frame()), 200)
        self.clock.now += 0.51
        self.assertEqual(self.throttle(self.frame(scan_ranges=None)), 0)
        for invalid in ([], [float('nan')], [float('inf')], [0.0]):
            with self.subTest(scan=invalid):
                self.assertEqual(self.throttle(self.frame(scan_ranges=invalid)), 0)
        self.assertEqual(self.throttle(self.frame()), 200)

    def test_confirmed_red_overrides_continuously_refreshed_evasion(self):
        outputs = [self.frame([perception.box(2)], evasion=True) for _ in range(3)]
        self.assertEqual(outputs[:2], ['TH:200 STN:1000\n'] * 2)
        self.assertEqual(outputs[2], 'TH:0 STN:-250\n')
        for _ in range(8):
            self.assertEqual(self.throttle(self.frame(evasion=True)), 0)

    def test_duplicate_green_frame_cannot_release_red_at_serial_output(self):
        self.red_stop()
        self.assertEqual(self.throttle(self.frame([perception.box(1)])), 0)
        repeated = self.sign.last_accepted_stamp
        for _ in range(3):
            self.assertEqual(self.throttle(self.frame([perception.box(1)] * 3, image_stamp=repeated)), 0)
        # The repeated deliveries supplied no votes; only two subsequent
        # independent green frames complete the still-live three-frame series.
        self.assertEqual(self.throttle(self.frame([perception.box(1)])), 0)
        self.assertEqual(self.throttle(self.frame([perception.box(1)])), 200)

    def test_conflicting_signal_immediately_stops_even_with_evasion(self):
        self.assertEqual(self.throttle(self.frame()), 200)
        self.assertEqual(self.throttle(self.frame([perception.box(1), perception.box(2)], evasion=True)), 0)
        self.assertEqual(self.throttle(self.frame()), 0)


if __name__ == '__main__':
    unittest.main()
