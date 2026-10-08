"""Check the actual commissioning launch graph without ROS or device access.

These tests inspect the nodes this launch would start; they cannot exclude an
unrelated vehicle controller already running in the same ROS domain.
"""
import ast
import os
from pathlib import Path
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[1]


def load_launch():
    class LaunchConfiguration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context[self.name]

    def declaration(name, default_value, description):
        return SimpleNamespace(kind='argument', name=name, default=default_value)

    def opaque(function):
        return SimpleNamespace(kind='opaque', function=function)

    def node(**kwargs):
        return SimpleNamespace(kind='node', **kwargs)

    source = ROOT / 's1_stack' / 'launch' / 'commissioning.launch.py'
    tree = ast.parse(source.read_text())
    tree.body = [item for item in tree.body
                 if not isinstance(item, (ast.Import, ast.ImportFrom))]
    scope = dict(__name__='commissioning_launch_test', os=os,
                 LaunchDescription=lambda actions: actions,
                 DeclareLaunchArgument=declaration, OpaqueFunction=opaque,
                 LaunchConfiguration=LaunchConfiguration, Node=node,
                 get_package_share_directory=lambda package: str(ROOT / package))
    exec(compile(tree, str(source), 'exec'), scope)
    description = scope['generate_launch_description']()
    defaults = {action.name: action.default for action in description
                if action.kind == 'argument'}
    callbacks = [action.function for action in description if action.kind == 'opaque']
    return defaults, callbacks


def graph(**overrides):
    defaults, callbacks = load_launch()
    defaults.update(overrides)
    nodes = []
    for callback in callbacks:
        nodes.extend(callback(defaults))
    return nodes


def node_for(nodes, executable):
    matches = [node for node in nodes if node.executable == executable]
    if len(matches) != 1:
        raise AssertionError(f'Expected one {executable}, got {len(matches)}')
    return matches[0]


def explicit_parameters(node):
    # Per-node dictionaries come after the YAML file, so these are the launch
    # overrides for the actual motion-critical settings tested below.
    parameters = {}
    for item in getattr(node, 'parameters', []):
        if isinstance(item, dict):
            parameters.update(item)
    return parameters


class CommissioningLaunchTests(unittest.TestCase):
    def test_default_profile_only_starts_sensor_drivers(self):
        nodes = graph()
        self.assertTrue(nodes)
        allowed = {('sllidar_ros2', 'sllidar_node'), ('usb_cam', 'usb_cam_node_exe')}
        self.assertTrue({(n.package, n.executable) for n in nodes} <= allowed)

    def test_read_only_profiles_cannot_start_serial_or_motion_mux(self):
        forbidden = {'serial_sender', 'cmd_mux', 'joy_to_twist', 'joy_node'}
        for profile in ('sensors', 'observe'):
            # Motion arguments must not accidentally enable actuation here.
            with self.subTest(profile=profile):
                nodes = graph(profile=profile, enable_evasion='true',
                              arduino_port='/must-not-be-opened', max_throttle='1000')
                self.assertFalse(forbidden & {node.executable for node in nodes})
                if profile == 'observe':
                    self.assertEqual(
                        {node.executable for node in nodes if node.package == 's1_stack'},
                        {'decision_auto'})

    def test_observe_commands_and_signals_are_isolated_from_normal_drive_topics(self):
        nodes = graph(profile='observe')
        expected = {f'/{name}': f'/s1_check/{name}' for name in (
            'cmd_lane', 'cmd_obs', 'cmd_auto', 'light_stop', 'cross_stop', 'lane_change_flag')}
        for executable in ('lane_trace', 'sign_detect', 'obs_evade', 'decision_auto'):
            with self.subTest(executable=executable):
                remappings = dict(node_for(nodes, executable).remappings)
                for source, target in expected.items():
                    self.assertEqual(remappings.get(source), target)
                self.assertNotIn('/scan', remappings)
                self.assertNotIn('/cam_lane/image_raw', remappings)
                self.assertNotIn('/cam_front/image_raw', remappings)

    def test_manual_profile_disables_auto_and_passes_serial_cap_and_ack_gate(self):
        nodes = graph(profile='manual')
        mux = explicit_parameters(node_for(nodes, 'cmd_mux'))
        serial = explicit_parameters(node_for(nodes, 'serial_sender'))
        self.assertIs(mux['allow_auto'], False)
        self.assertEqual(mux['initial_mode'], 2)
        self.assertEqual(serial['max_throttle'], 200)
        self.assertIs(serial['require_calibration_ack'], True)
        self.assertFalse({'sllidar_node', 'usb_cam_node_exe', 'decision_auto',
                          'obs_evade'} & {node.executable for node in nodes})
        for limit in (0, 75, 1000):
            with self.subTest(limit=limit):
                configured = graph(profile='manual', max_throttle=str(limit))
                serial = explicit_parameters(node_for(configured, 'serial_sender'))
                self.assertEqual(serial['max_throttle'], limit)

    def test_auto_requires_scan_and_joystick_but_timed_evasion_is_opt_in(self):
        nodes = graph(profile='auto')
        self.assertNotIn('obs_evade', {node.executable for node in nodes})
        decision = explicit_parameters(node_for(nodes, 'decision_auto'))
        mux = explicit_parameters(node_for(nodes, 'cmd_mux'))
        self.assertIs(decision['require_scan'], True)
        self.assertIs(decision['allow_evasion'], False)
        self.assertIs(mux['allow_auto'], True)
        self.assertIs(mux['require_joy_for_auto'], True)
        enabled = graph(profile='auto', enable_evasion='true')
        self.assertEqual(node_for(enabled, 'obs_evade').package, 'obs_evade')
        self.assertIs(explicit_parameters(node_for(enabled, 'decision_auto'))['allow_evasion'], True)

    def test_invalid_profile_throttle_and_evasion_values_are_rejected(self):
        cases = ({'profile': 'drive'}, {'profile': ''}, {'max_throttle': '-1'},
                 {'max_throttle': '1001'}, {'max_throttle': '0.5'},
                 {'max_throttle': 'fast'}, {'enable_evasion': 'yes'})
        for overrides in cases:
            with self.subTest(overrides=overrides):
                with self.assertRaises(ValueError):
                    graph(**overrides)

    def test_local_python_executables_have_registered_existing_main_functions(self):
        configured = {}
        for profile in ('sensors', 'observe', 'manual', 'auto'):
            for node in graph(profile=profile, enable_evasion='true'):
                configured[(node.package, node.executable)] = node
        for package, executable in configured:
            setup_path = ROOT / package / 'setup.py'
            if not setup_path.is_file():
                # External joy/usb_cam and the C++ lidar are checked on target.
                continue
            with self.subTest(package=package, executable=executable):
                setup_tree = ast.parse(setup_path.read_text())
                setup_call = next(
                    node for node in ast.walk(setup_tree)
                    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                    and node.func.id == 'setup')
                entries = ast.literal_eval(next(
                    kw.value for kw in setup_call.keywords if kw.arg == 'entry_points'))
                targets = dict(entry.split('=', 1) for entry in entries['console_scripts'])
                targets = {name.strip(): target.strip() for name, target in targets.items()}
                self.assertIn(executable, targets)
                module, function = targets[executable].split(':')
                source = ROOT / package / (module.replace('.', '/') + '.py')
                self.assertTrue(source.is_file(), str(source))
                definitions = ast.parse(source.read_text()).body
                self.assertTrue(any(isinstance(item, ast.FunctionDef) and
                                    item.name == function for item in definitions))


if __name__ == '__main__':
    unittest.main()
