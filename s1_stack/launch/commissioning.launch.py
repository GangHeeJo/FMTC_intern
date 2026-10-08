"""Staged field checks. Default sensors profile never opens the Arduino port."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    profile = value('profile')
    if profile not in ('sensors', 'observe', 'manual', 'auto'):
        raise ValueError('profile must be sensors, observe, manual, or auto')
    limit = int(value('max_throttle'))
    if not 0 <= limit <= 1000:
        raise ValueError('max_throttle must be an integer from 0 through 1000')
    evasion = value('enable_evasion').lower()
    if evasion not in ('true', 'false'):
        raise ValueError('enable_evasion must be true or false')
    params = os.path.join(get_package_share_directory('s1_stack'), 'config', 's1_stack.yaml')
    nodes = []
    if profile != 'manual':
        nodes.append(Node(
            package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
            output='screen', parameters=[params, {
                'serial_port': value('lidar_port'),
                'serial_baudrate': int(value('lidar_baud')),
            }]))
        for name in ('lane', 'front'):
            nodes.append(Node(
                package='usb_cam', executable='usb_cam_node_exe',
                name='usb_cam_' + name, namespace='cam_' + name, output='screen',
                parameters=[{
                    'video_device': value(name + '_camera'),
                    'image_width': 640, 'image_height': 480,
                    'framerate': 30.0, 'pixel_format': value('pixel_format'),
                }]))
    if profile in ('observe', 'auto'):
        remappings = ([(f'/{name}', f'/s1_check/{name}') for name in (
            'cmd_lane', 'cmd_obs', 'cmd_auto', 'light_stop', 'cross_stop', 'lane_change_flag')]
            if profile == 'observe' else [])
        sign_params = {'debug_view': False}
        if value('model_path'):
            sign_params['model_path'] = value('model_path')
        nodes.extend([
            Node(package='lane_trace', executable='lane_trace', name='lane_trace',
                 output='screen', parameters=[{'debug_view': False}], remappings=remappings),
            Node(package='sign_detect', executable='sign_detect', name='sign_detect',
                 output='screen', parameters=[sign_params], remappings=remappings),
            Node(package='s1_stack', executable='decision_auto', name='decision_auto',
                 output='screen', parameters=[params, {
                     'require_scan': True, 'allow_evasion': profile == 'observe' or evasion == 'true',
                 }], remappings=remappings),
        ])
        # Observation includes the existing detector; track actuation is opt-in.
        if profile == 'observe' or evasion == 'true':
            nodes.append(Node(package='obs_evade', executable='obs_evade',
                              name='obs_evade', output='screen', remappings=remappings))
    if profile in ('manual', 'auto'):
        nodes.extend([
            Node(package='joy', executable='joy_node', name='joy_node',
                 output='screen', parameters=[params]),
            Node(package='s1_stack', executable='joy_to_twist', name='joy_to_twist',
                 output='screen', parameters=[params]),
            Node(package='s1_stack', executable='cmd_mux', name='cmd_mux',
                 output='screen', parameters=[params, {
                     'initial_mode': 2, 'allow_auto': profile == 'auto',
                     'require_joy_for_auto': True, 'joy_timeout_s': 0.5,
                 }]),
            Node(package='s1_stack', executable='serial_sender', name='serial_sender',
                 output='screen', parameters=[params, {
                     'port': value('arduino_port'), 'max_throttle': limit,
                     'require_calibration_ack': True, 'dry_run': False,
                 }]),
        ])
    return nodes


def generate_launch_description():
    arguments = {
        'profile': ('sensors', 'sensors/observe: no Arduino; manual/auto: opens Arduino'),
        'arduino_port': ('/dev/arduino_mega', 'Existing verified Arduino device'),
        'lidar_port': ('/dev/rplidar', 'Existing RPLIDAR device'),
        'lidar_baud': ('256000', 'Verify against the installed lidar, not its nickname'),
        'lane_camera': ('/dev/cam_lane', 'Verified lane camera device'),
        'front_camera': ('/dev/cam_front', 'Verified forward camera device'),
        'pixel_format': ('mjpeg2rgb', 'Use a pixel format supported by installed usb_cam'),
        'model_path': ('', 'Empty uses packaged best_0808.pt; explicit absolute path allowed'),
        'max_throttle': ('200', 'Absolute TH command cap; NOT a measured speed'),
        'enable_evasion': ('false', 'auto profile: enable unvalidated timed evasion explicitly'),
    }
    return LaunchDescription([
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, description) in arguments.items()
    ] + [OpaqueFunction(function=setup)])
