import importlib.util
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[1] / 'scripts' / 's1_topic_check.py'
spec = importlib.util.spec_from_file_location('topic_check', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TopicCheckTests(unittest.TestCase):
    def records(self, profile):
        return {topic: {'count': 10, 'publishers': ['node'],
                        'last_age_s': 0.01,
                        'last': {'data': True, 'usable_returns': 10}}
                for topic in module.topic_spec(profile)}

    def test_quiet_stop_command_is_not_missing_sensor(self):
        records = self.records('manual')
        records['/cmd_out']['count'] = 0
        records['/serial_tx']['count'] = 0
        self.assertEqual(module.findings('manual', records), [])

    def test_duplicate_control_publisher_and_unready_board_fail(self):
        records = self.records('auto')
        records['/cmd_auto']['publishers'].append('gap_follow')
        records['/serial_ready']['last'] = {'data': False}
        self.assertEqual(len(module.findings('auto', records)), 2)

    def test_observation_detects_existing_motion_stack(self):
        records = self.records('observe')
        records['/cmd_out'] = {'publishers': ['unexpected_mux']}
        self.assertTrue(any('unexpected motion' in issue
                            for issue in module.findings('observe', records)))

    def test_missing_sensor_and_invalid_ranges_are_reported(self):
        records = self.records('sensors')
        records['/scan']['count'] = 0
        records['/scan']['last'] = {}
        records['/cam_front/image_raw']['publishers'] = []
        self.assertEqual(len(module.findings('sensors', records)), 3)

    def test_early_messages_do_not_hide_later_sensor_stall(self):
        records = self.records('sensors')
        records['/cam_lane/image_raw']['last_age_s'] = 3.0
        self.assertTrue(any('stale' in issue for issue in module.findings('sensors', records)))


if __name__ == '__main__':
    unittest.main()
