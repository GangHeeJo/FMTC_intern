"""Host callback regressions; no ROS transport, inference, cameras or hardware.

Run: python3 -m unittest discover -s tests -v
"""
import ast
import copy
from collections import deque
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


ROOT = Path(__file__).resolve().parents[1]
SENSOR_QOS = object()


class Twist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0.0)
        self.angular = SimpleNamespace(z=0.0)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


class Bridge:
    def imgmsg_to_cv2(self, message, **kwargs):
        return message


class CvBridgeError(Exception):
    pass


class Coordinates(list):
    def tolist(self):
        return list(self)


class Model:
    def __init__(self, path):
        self.path = path
        self.boxes = []
        self.plot = Mock(return_value='debug image')
        self.calls = 0
        self.on_inference = lambda: None

    def to(self, device):
        return self

    def __call__(self, *args, **kwargs):
        self.calls += 1
        self.on_inference()
        return [SimpleNamespace(boxes=self.boxes, plot=self.plot)]


def box(class_id, confidence=0.9, coords=(200, 100, 355, 200)):
    return SimpleNamespace(cls=[class_id], conf=[confidence],
                           xyxy=[Coordinates(coords)])


def load_source(relative, overrides=None):
    """Execute actual source definitions with local dependency doubles."""
    overrides = overrides or {}
    clock = SimpleNamespace(ros=10.0, wall=100.0)

    class Node:
        def __init__(self, name):
            self.params = {}
            self.subscriptions = []

        def declare_parameter(self, name, default):
            self.params[name] = overrides.get(name, default)

        def get_parameter(self, name):
            value = self.params[name]
            return SimpleNamespace(
                value=value,
                get_parameter_value=lambda: SimpleNamespace(string_value=value))

        def create_publisher(self, *args):
            return Publisher()

        def create_subscription(self, *args):
            self.subscriptions.append(args)

        def get_logger(self):
            return SimpleNamespace(info=Mock(), warn=Mock())

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(clock.ros*1e9)))

        def destroy_node(self):
            pass

    gui = SimpleNamespace(imshow=Mock(), waitKey=Mock(),
                          destroyAllWindows=Mock(), flip=lambda image, axis: image,
                          error=RuntimeError)
    scope = dict(__name__='perception_callback_test', Node=Node, Twist=Twist,
                 Bool=SimpleNamespace, Image=SimpleNamespace, CvBridge=Bridge, CvBridgeError=CvBridgeError,
                 YOLO=Model, torch=SimpleNamespace(cuda=SimpleNamespace(
                     is_available=lambda: False)), cv2=gui, deque=deque, Path=Path,
                 np=SimpleNamespace(degrees=math.degrees), math=math,
                 time=SimpleNamespace(monotonic=lambda: clock.wall), test_clock=clock,
                 qos_profile_sensor_data=SENSOR_QOS,
                 get_package_share_directory=lambda package: str(ROOT / package))
    source = ROOT / relative
    tree = ast.parse(source.read_text())
    tree.body = [item for item in tree.body
                 if not isinstance(item, (ast.Import, ast.ImportFrom))]
    exec(compile(tree, str(source), 'exec'), scope)
    return scope


class SignalSafetyTests(unittest.TestCase):
    def setUp(self):
        self.scope = load_source('sign_detect/sign_detect/sign_detect.py')
        self.node = self.scope['VisionPercept']()

    def frame(self, *boxes, stamp=None, advance=0.03):
        clock = self.scope['test_clock']
        clock.ros += advance
        clock.wall += advance
        stamp = clock.ros if stamp is None else stamp
        seconds = math.floor(stamp)
        msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
            sec=seconds, nanosec=round((stamp-seconds)*1e9))))
        self.node.model.boxes = list(boxes)
        self.accepted = self.node.image_callback(msg)
        return self.node.pub_light.messages[-1].data

    def histories(self):
        return tuple(tuple(h) for h in (self.node.cw_history, self.node.tl_history,
                                        self.node.green_history))

    def red_stop(self):
        self.assertFalse(self.frame(box(2)))
        self.assertFalse(self.frame(box(2)))
        self.assertTrue(self.frame(box(2)))

    def test_red_latches_through_missing_detection_until_confirmed_green(self):
        self.red_stop()
        for _ in range(20):
            self.assertTrue(self.frame())
        self.assertTrue(self.frame(box(1)))
        self.assertTrue(self.frame(box(1)))
        self.assertFalse(self.frame(box(1)))

    def test_yellow_and_red_votes_combine_within_five_frames(self):
        self.assertFalse(self.frame(box(3)))
        self.assertFalse(self.frame())
        self.assertFalse(self.frame(box(2)))
        self.assertFalse(self.frame())
        self.assertTrue(self.frame(box(3)))

    def test_conflicting_red_wins_and_discards_accumulated_green_votes(self):
        self.red_stop()
        self.assertTrue(self.frame(box(1)))
        self.assertTrue(self.frame(box(1)))
        self.assertTrue(self.frame(box(1), box(2)))
        self.assertTrue(self.frame(box(1)))
        self.assertTrue(self.frame(box(1)))
        self.assertFalse(self.frame(box(1)))

    def test_conflicting_heads_stop_immediately_even_before_initial_red_latch(self):
        self.assertTrue(self.frame(box(1), box(2)))
        self.assertTrue(self.node.light_stop_latched)
        self.assertTrue(self.frame(box(1)))
        self.assertTrue(self.frame(box(1)))
        self.assertFalse(self.frame(box(1)))

    def test_green_release_requires_consecutive_new_green_only_frames(self):
        self.red_stop()
        self.assertTrue(self.frame(box(1)))
        self.assertTrue(self.frame(box(1)))
        self.assertTrue(self.frame())
        self.assertTrue(self.frame(box(1)))
        self.assertTrue(self.frame(box(1)))
        self.assertFalse(self.frame(box(1)))

    def test_duplicate_out_of_order_expired_and_future_frames_never_mutate_votes(self):
        self.red_stop()
        self.assertTrue(self.frame(box(1)))
        accepted_stamp = self.node.last_accepted_stamp
        before = self.histories()
        calls = self.node.model.calls
        for stamp in (accepted_stamp, accepted_stamp-0.01,
                      self.scope['test_clock'].ros-0.6, self.scope['test_clock'].ros+0.2):
            with self.subTest(stamp=stamp):
                self.assertTrue(self.frame(box(1), stamp=stamp, advance=0))
                self.assertIs(self.accepted, False)
                self.assertTrue(self.node.pub_cross.messages[-1].data)
                self.assertEqual(self.histories(), before)
                self.assertEqual(self.node.last_accepted_stamp, accepted_stamp)
                self.assertEqual(self.node.model.calls, calls)
        self.assertTrue(self.frame(box(1)))
        self.assertFalse(self.frame(box(1)))

    def test_rejected_input_cannot_refresh_a_previous_permissive_bool(self):
        self.assertFalse(self.frame())
        stamp = self.node.last_accepted_stamp
        self.assertTrue(self.frame(stamp=stamp, advance=0))
        self.assertIs(self.accepted, False)
        self.assertTrue(self.node.pub_cross.messages[-1].data)
        self.assertFalse(self.node.light_stop_latched)

    def test_source_or_wall_expiry_and_clock_reversal_during_inference_discard_all_votes(self):
        for failure in ('source_age', 'wall_only', 'clock_backwards'):
            with self.subTest(failure=failure):
                self.setUp()
                self.red_stop()
                self.frame(box(1))
                self.frame(box(1))
                before = self.histories()
                last = self.node.last_accepted_stamp
                clock = self.scope['test_clock']

                def delay():
                    if failure == 'source_age':
                        clock.ros += 0.51
                        clock.wall += 0.51
                    elif failure == 'wall_only':
                        clock.wall += 0.51
                    else:
                        clock.ros -= 0.01

                self.node.model.on_inference = delay
                self.assertTrue(self.frame(box(1)))
                self.assertIs(self.accepted, False)
                self.assertEqual(self.histories(), before)
                self.assertEqual(self.node.last_accepted_stamp, last)
                self.assertTrue(self.node.light_stop_latched)
                self.assertTrue(self.node.pub_cross.messages[-1].data)

    def test_long_source_or_frozen_clock_gap_resets_votes_but_preserves_red_latch(self):
        for failure in ('source_gap', 'wall_gap'):
            with self.subTest(failure=failure):
                self.setUp()
                self.red_stop()
                self.frame(box(1), box(0))
                self.frame(box(1), box(0))
                clock = self.scope['test_clock']
                clock.wall += 0.6
                if failure == 'source_gap':
                    clock.ros += 0.6
                self.assertTrue(self.frame(box(1), box(0)))
                self.assertEqual(tuple(self.node.green_history), (True,))
                self.assertEqual(tuple(self.node.tl_history), (False,))
                self.assertEqual(tuple(self.node.cw_history), (True,))
                self.assertFalse(self.node.pub_cross.messages[-1].data)
                self.assertTrue(self.frame(box(1)))
                self.assertFalse(self.frame(box(1)))

    def test_multiple_boxes_in_one_fresh_frame_are_only_one_temporal_vote(self):
        self.red_stop()
        self.assertTrue(self.frame(box(1), box(1), box(1)))
        self.assertEqual(tuple(self.node.green_history), (True,))
        self.assertTrue(self.frame(box(1)))
        self.assertFalse(self.frame(box(1)))

    def test_conversion_inference_and_nonfinite_detector_errors_fail_stop_without_votes(self):
        for failure in ('conversion', 'inference', 'nonfinite_box'):
            with self.subTest(failure=failure):
                self.setUp()
                self.red_stop()
                before = self.histories()
                if failure == 'conversion':
                    self.node.bridge.imgmsg_to_cv2 = Mock(side_effect=CvBridgeError('bad encoding'))
                elif failure == 'inference':
                    self.node.model.on_inference = Mock(side_effect=RuntimeError('inference failed'))
                bad = box(1, coords=(200, 100, float('nan'), 300)) if failure == 'nonfinite_box' else box(1)
                self.assertTrue(self.frame(bad))
                self.assertIs(self.accepted, False)
                self.assertEqual(self.histories(), before)
                self.assertTrue(self.node.pub_cross.messages[-1].data)

    def test_green_before_red_cannot_release_new_stop(self):
        for _ in range(5):
            self.assertFalse(self.frame(box(1)))
        self.red_stop()
        self.assertTrue(self.frame())
        self.assertTrue(self.frame(box(1)))
        self.assertTrue(self.frame(box(1)))
        self.assertFalse(self.frame(box(1)))

    def test_small_low_confidence_and_old_green_votes_do_not_release(self):
        self.red_stop()
        for _ in range(6):
            self.assertTrue(self.frame(box(1, confidence=0.3)))
            self.assertTrue(self.frame(box(1, coords=(200, 100, 354, 200))))
        self.assertTrue(self.frame(box(1)))
        self.assertTrue(self.frame(box(1)))
        for _ in range(5):
            self.assertTrue(self.frame())
        self.assertTrue(self.frame(box(1)))

    def test_crosswalk_stop_is_preserved_until_detection_clears(self):
        for _ in range(10):
            self.frame(box(0))
        self.assertTrue(self.node.pub_cross.messages[-1].data)
        self.frame()
        self.frame()
        self.assertTrue(self.node.pub_cross.messages[-1].data)
        self.frame()
        self.assertFalse(self.node.pub_cross.messages[-1].data)

    def test_headless_default_skips_plot_and_windows_and_uses_sensor_qos(self):
        self.frame(box(2))
        self.node.model.plot.assert_not_called()
        self.scope['cv2'].imshow.assert_not_called()
        self.scope['cv2'].waitKey.assert_not_called()
        self.assertIs(self.node.subscriptions[0][-1], SENSOR_QOS)

    def test_debug_view_is_opt_in(self):
        self.scope = load_source('sign_detect/sign_detect/sign_detect.py',
                                 {'debug_view': True})
        self.node = self.scope['VisionPercept']()
        self.frame()
        self.node.model.plot.assert_called_once()
        self.scope['cv2'].imshow.assert_called_once()

    def test_model_default_uses_package_share_and_missing_override_fails(self):
        self.assertEqual(self.node.model.path,
                         str(ROOT / 'sign_detect' / 'best_0808.pt'))
        scope = load_source('sign_detect/sign_detect/sign_detect.py',
                            {'model_path': str(ROOT / 'sign_detect' / 'not-a-model.pt')})
        with self.assertRaisesRegex(FileNotFoundError, 'model_path'):
            scope['VisionPercept']()


class LaneSafetyTests(unittest.TestCase):
    def setUp(self):
        self.scope = load_source('lane_trace/lane_trace/lane_trace.py')
        self.node = self.scope['LaneMaskingNode']()

    def frame(self, steer):
        self.scope['lane_detect'] = lambda image: (None, None, steer)
        self.node.image_callback('image')
        return self.node.cmd_pub.messages[-1]

    def test_no_lane_at_start_stops_then_valid_lane_recovers(self):
        self.assertEqual(self.frame(None).linear.x, 0.0)
        command = self.frame(0.2)
        self.assertEqual((command.linear.x, command.angular.z), (0.5, -500.0))

    def test_lane_loss_stops_throttle_and_keeps_previous_steering(self):
        self.frame(0.2)
        command = self.frame(None)
        self.assertEqual((command.linear.x, command.angular.z), (0.0, -500.0))
        self.assertEqual(self.frame(None).linear.x, 0.0)

    def test_headless_default_skips_windows_and_uses_sensor_qos(self):
        self.frame(0.0)
        self.scope['cv2'].imshow.assert_not_called()
        self.scope['cv2'].waitKey.assert_not_called()
        self.assertIs(self.node.subscriptions[0][-1], SENSOR_QOS)

    def test_headless_shutdown_does_not_call_opencv_gui_for_either_node(self):
        for relative in ('lane_trace/lane_trace/lane_trace.py',
                         'sign_detect/sign_detect/sign_detect.py'):
            with self.subTest(source=relative):
                scope = load_source(relative)
                scope['rclpy'] = SimpleNamespace(init=Mock(), spin=Mock(), shutdown=Mock())
                scope['main']()
                scope['cv2'].destroyAllWindows.assert_not_called()


if __name__ == '__main__':
    unittest.main()
