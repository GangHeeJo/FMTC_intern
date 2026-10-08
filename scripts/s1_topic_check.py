#!/usr/bin/env python3
"""Bounded, subscribe-only ROS2 field check. Never sends control commands."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time


def topic_spec(profile):
    spec = {}
    if profile != 'manual':
        spec.update({'/scan': ('LaserScan', True),
                     '/cam_lane/image_raw': ('Image', True),
                     '/cam_front/image_raw': ('Image', True)})
    if profile in ('observe', 'auto'):
        spec.update({'/cmd_lane': ('Twist', True), '/light_stop': ('Bool', True),
                     '/cross_stop': ('Bool', True), '/cmd_auto': ('Twist', True)})
    if profile in ('manual', 'auto'):
        spec.update({'/joy': ('Joy', True), '/cmd_manual': ('Twist', True),
                     '/cmd_out': ('Twist', False), '/serial_ready': ('Bool', True),
                     '/serial_tx': ('String', False)})
    if profile == 'observe':
        spec = {(('/s1_check' + topic) if topic in (
            '/cmd_lane', '/light_stop', '/cross_stop', '/cmd_auto') else topic): value
                for topic, value in spec.items()}
    return spec


def describe(msg, kind):
    if kind == 'Twist':
        return {'throttle_fraction': msg.linear.x, 'steering_stn': msg.angular.z}
    if kind == 'Image':
        return {'width': msg.width, 'height': msg.height, 'encoding': msg.encoding}
    if kind == 'LaserScan':
        valid = [float(r) for r in msg.ranges
                 if math.isfinite(r) and max(0, msg.range_min) < r <= msg.range_max]
        return {'frame': msg.header.frame_id, 'samples': len(msg.ranges),
                'usable_returns': len(valid), 'nearest_any_direction_m': min(valid) if valid else None,
                'angle_min': msg.angle_min, 'angle_max': msg.angle_max}
    if kind == 'Joy':
        return {'axes': list(msg.axes), 'buttons': list(msg.buttons)}
    return {'data': msg.data}


def findings(profile, records):
    issues = []
    for topic, (_, needs_messages) in topic_spec(profile).items():
        record = records[topic]
        if len(record['publishers']) != 1:
            issues.append(f'{topic}: expected one publisher, saw {len(record["publishers"])}')
        if needs_messages and record['count'] == 0:
            issues.append(f'{topic}: no message received in the check window')
        elif needs_messages:
            limit = 1.0 if topic.endswith(('light_stop', 'cross_stop')) else 0.5
            if record.get('last_age_s') is None or record['last_age_s'] >= limit:
                issues.append(f'{topic}: last message stale at end of window (limit {limit}s)')
    if '/serial_ready' in records and not records['/serial_ready']['last'].get('data', False):
        issues.append('/serial_ready: Arduino calibration readiness not confirmed')
    if '/scan' in records and records['/scan']['last'].get('usable_returns', 0) == 0:
        issues.append('/scan: no usable ranges in last scan')
    if profile in ('sensors', 'observe'):
        for topic in ('/cmd_out', '/serial_tx'):
            if records.get(topic, {}).get('publishers'):
                issues.append(f'{topic}: unexpected motion stack in this ROS domain')
    return issues


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=('sensors', 'observe', 'manual', 'auto'), default='sensors')
    parser.add_argument('--seconds', type=float, default=8.0)
    parser.add_argument('--output', type=Path, default=Path('s1_checks'))
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 2 <= args.seconds <= 60:
        parser.error('--seconds must be between 2 and 60')
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, Joy, LaserScan
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Bool, String
    except ImportError as exc:
        parser.exit(2, f'ROS Python unavailable; source the existing ROS/workspace environment first: {exc}\n')
    types = dict(Image=Image, Joy=Joy, LaserScan=LaserScan, Twist=Twist, Bool=Bool, String=String)
    rclpy.init(args=[])
    node = Node('s1_read_only_topic_check')
    spec = topic_spec(args.profile)
    observed = dict(spec)
    if args.profile in ('sensors', 'observe'):
        observed.update({'/cmd_out': ('Twist', False), '/serial_tx': ('String', False)})
    records = {topic: {'count': 0, 'last': {}, 'publishers': [], 'first_at': None,
                       'last_at': None, 'max_gap_s': 0.0}
               for topic in observed}
    def callback(topic, kind):
        def receive(msg):
            record = records[topic]
            now = time.monotonic()
            if record['last_at'] is not None:
                record['max_gap_s'] = max(record['max_gap_s'], now - record['last_at'])
            if record['first_at'] is None:
                record['first_at'] = now
            record['last_at'] = now
            record['count'] += 1
            record['last'] = describe(msg, kind)
        return receive
    try:
        for topic, (kind, _) in observed.items():
            node.create_subscription(types[kind], topic, callback(topic, kind), qos_profile_sensor_data)
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.2, end - time.monotonic())))
        for topic, record in records.items():
            record['publishers'] = [f'{info.node_namespace.rstrip("/")}/{info.node_name}'
                                    for info in node.get_publishers_info_by_topic(topic)]
            span = ((record['last_at'] - record['first_at'])
                    if record['count'] > 1 else 0)
            record['received_hz'] = (record['count'] - 1) / span if span else 0
            record['last_age_s'] = time.monotonic() - record['last_at'] if record['count'] else None
        issues = findings(args.profile, records)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    args.output.mkdir(parents=True, exist_ok=True)
    path = args.output / f'{stamp}_{args.profile}_topics.json'
    path.write_text(json.dumps({'profile': args.profile, 'window_s': args.seconds,
                                'issues': issues, 'topics': records}, indent=2) + '\n')
    for topic, record in records.items():
        print(f'{topic}: {record["count"]} messages, {record["received_hz"]:.1f} Hz, '
              f'publishers={len(record["publishers"])}; last={record["last"]}')
    for issue in issues:
        print('CHECK:', issue)
    print(f'Saved: {path}')
    print('This checks topic delivery/graph only; it does not approve physical driving or recognition accuracy.')
    return 1 if issues else 0


if __name__ == '__main__':
    raise SystemExit(main())
