"""Host-only callback tests; ROS transport, USB and hardware are not simulated.

Run: python3 -m unittest discover -s tests -v
"""
import ast
import copy
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1] / 's1_stack' / 's1_stack'


class Twist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0.0)
        self.angular = SimpleNamespace(z=0.0)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


class Node:
    def __init__(self, name):
        self.params = {}

    def declare_parameter(self, name, value):
        self.params[name] = value

    def get_parameter(self, name):
        return SimpleNamespace(value=self.params[name])

    def create_publisher(self, *args):
        return Publisher()

    def create_subscription(self, *args):
        pass

    def create_timer(self, *args):
        pass

    def get_logger(self):
        return SimpleNamespace(info=lambda *args: None)


class Clock:
    now = 10.0

    def monotonic(self):
        return self.now


def load_node(filename, name, clock):
    # Execute the actual node definitions with local transport/message doubles.
    # Do not replace global ROS modules: these tests also coexist with ROS tests.
    tree = ast.parse((ROOT / filename).read_text())
    tree.body = [item for item in tree.body
                 if not isinstance(item, (ast.Import, ast.ImportFrom))]
    scope = dict(__name__='callback_test', Node=Node, Twist=Twist,
                 Joy=SimpleNamespace, Bool=SimpleNamespace,
                 UInt8=SimpleNamespace, time=clock)
    scope.update(math=math, LaserScan=SimpleNamespace, qos_profile_sensor_data=object())
    exec(compile(tree, str(ROOT / filename), 'exec'), scope)
    return scope[name]()


def command(speed=0.5, steer=100.0):
    msg = Twist()
    msg.linear.x, msg.angular.z = speed, steer
    return msg


class WatchdogTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.decision = load_node('decision_auto.py', 'MotionDecision', self.clock)

    def ready(self):
        self.decision.lane_cb(command())
        self.decision.light_cb(SimpleNamespace(data=False))
        self.decision.cross_cb(SimpleNamespace(data=False))
        self.scan()

    def scan(self, ranges=(1.0, 2.0)):
        self.decision.scan_cb(SimpleNamespace(
            ranges=ranges, range_min=0.1, range_max=10.0, angle_increment=0.01))

    def output(self):
        self.decision.publish_decision()
        return self.decision.pub_auto.messages[-1]

    def test_startup_and_missing_signal_stop(self):
        self.assertEqual(self.output().linear.x, 0)
        self.decision.lane_cb(command())
        self.decision.light_cb(SimpleNamespace(data=False))
        self.assertEqual(self.output().linear.x, 0)
        self.decision.cross_cb(SimpleNamespace(data=False))
        self.scan()
        self.assertEqual(self.output().linear.x, 0.5)

    def test_lane_stall_stops_and_fresh_input_recovers(self):
        self.ready()
        self.assertEqual(self.output().linear.x, 0.5)
        self.clock.now += 0.51
        self.assertEqual(self.output().linear.x, 0)
        self.decision.lane_cb(command())
        self.scan()
        self.assertEqual(self.output().linear.x, 0.5)

    def test_each_signal_stream_must_stay_live(self):
        for stale in ('light', 'cross'):
            with self.subTest(stale=stale):
                self.ready()
                self.clock.now += 1.01
                self.decision.lane_cb(command())
                self.scan()
                if stale == 'light':
                    self.decision.cross_cb(SimpleNamespace(data=False))
                else:
                    self.decision.light_cb(SimpleNamespace(data=False))
                self.assertEqual(self.output().linear.x, 0)

    def test_both_stop_flags_override_live_evasion(self):
        for callback in ('light_cb', 'cross_cb'):
            with self.subTest(callback=callback):
                self.ready()
                self.decision.obs_cb(command(0.7, 1000))
                getattr(self.decision, callback)(SimpleNamespace(data=True))
                self.assertEqual(self.output().linear.x, 0)

    def test_evasion_expires_back_to_lane(self):
        self.ready()
        self.decision.obs_cb(command(0.7, 1000))
        self.assertEqual(self.output().angular.z, 1000)
        self.clock.now += 0.21
        self.assertEqual(self.output().angular.z, 100)

    def test_live_evasion_cannot_hide_stale_lane(self):
        self.ready()
        self.clock.now += 0.51
        self.decision.obs_cb(command(0.7, 1000))
        self.scan()
        self.assertEqual(self.output().linear.x, 0)

    def test_disabled_evasion_ignores_stray_obstacle_commands(self):
        self.ready()
        self.decision.allow_evasion = False
        self.decision.obs_cb(command(0.7, 1000))
        self.assertEqual(self.output().angular.z, 100)

    def test_lidar_missing_stale_and_unusable_stop(self):
        self.ready()
        self.decision.last_scan_time = None
        self.assertEqual(self.output().linear.x, 0)
        self.scan()
        self.assertEqual(self.output().linear.x, 0.5)
        self.clock.now += 0.51
        self.decision.lane_cb(command())
        self.assertEqual(self.output().linear.x, 0)
        for ranges in ([], [float('nan')], [float('inf')], [0.0], [-1.0]):
            self.scan(ranges)
            self.assertEqual(self.output().linear.x, 0)

    def test_lane_loss_overrides_live_evasion(self):
        self.ready()
        self.decision.obs_cb(command(0.7, 1000))
        self.decision.lane_cb(command(0.0, 100))
        self.assertEqual(self.output().linear.x, 0)

    def test_manual_profile_cannot_select_auto(self):
        mux = load_node('cmd_mux.py', 'CmdMux', self.clock)
        mux.allow_auto = False
        mux.on_joy(SimpleNamespace(buttons=[1, 0, 0, 0]))
        self.assertEqual(mux.mode, 2)

    def test_auto_joystick_loss_latches_stop(self):
        mux = load_node('cmd_mux.py', 'CmdMux', self.clock)
        mux.require_joy_for_auto = True
        mux.mode = 1
        mux.on_auto(command())
        mux.on_timer()
        self.assertEqual(mux.mode, 2)
        mux.on_joy(SimpleNamespace(buttons=[1, 0, 0, 0]))
        self.clock.now += 0.21
        mux.on_auto(command())
        mux.on_timer()
        self.assertEqual(mux.mode, 1)
        self.clock.now += 0.51
        mux.on_auto(command())
        mux.on_timer()
        self.assertEqual(mux.mode, 2)
        self.assertEqual(mux.pub_cmd.messages[-1].linear.x, 0)

    def test_joy_stall_propagates_stop_through_mux(self):
        joy = load_node('joy_to_twist.py', 'JoyToTwist', self.clock)
        mux = load_node('cmd_mux.py', 'CmdMux', self.clock)
        mux.mode = 0
        joy.on_joy(SimpleNamespace(axes=[0.5, 0, 0, 1.0]))
        joy.on_timer()
        mux.on_manual(joy.pub.messages[-1])
        mux.on_timer()
        self.assertEqual(mux.pub_cmd.messages[-1].linear.x, 1.0)
        self.clock.now += 0.51
        joy.on_timer()
        mux.on_manual(joy.pub.messages[-1])
        mux.on_timer()
        self.assertEqual(mux.pub_cmd.messages[-1].linear.x, 0)
        self.assertEqual(mux.pub_cmd.messages[-1].angular.z, 0)

    def test_manual_publisher_stall_stops_mux(self):
        mux = load_node('cmd_mux.py', 'CmdMux', self.clock)
        mux.mode = 0
        mux.on_manual(command())
        self.clock.now += 0.51
        mux.on_timer()
        self.assertEqual(mux.pub_cmd.messages[-1].linear.x, 0)

    def test_auto_timeout_latches_stop_instead_of_manual(self):
        mux = load_node('cmd_mux.py', 'CmdMux', self.clock)
        mux.mode = 1
        mux.on_auto(command())
        self.clock.now += 0.31
        mux.on_manual(command(1.0))
        mux.on_timer()
        self.assertEqual(mux.mode, 2)
        self.assertEqual(mux.pub_cmd.messages[-1].linear.x, 0)
        mux.on_auto(command())
        mux.on_timer()
        self.assertEqual(mux.mode, 2)
        self.assertEqual(mux.pub_cmd.messages[-1].linear.x, 0)


if __name__ == '__main__':
    unittest.main()
