#!/usr/bin/env python3
"""rosbag 한 개를 0.5초 단위 텍스트 타임라인으로 요약 (10/21 회피·신호등 분석에 씀).

사용:
    source /opt/ros/humble/setup.bash
    python3 scripts/bag_analysis/timeline.py ~/bags/obs_175637/*.db3

출력
  == events   : /drive_mode(0 수동 1 자동 2 정지), /lane_changing, /lane_change_flag(True = 1차선 쪽),
                /crosswalk_active 가 바뀐 시각
  == timeline : 0.5초마다
      out   /cmd_out (속도, 조향)  조향 + = 왼쪽, - = 오른쪽
      lane  /cmd_lane (lane_trace가 원한 명령)
      obs   /cmd_obs (차선 변경 중일 때만 나옴)
      R a/b /light_stop True 개수 / 받은 개수 (빨간·노란불 정지)
      G a/b /light_green
      sl    /stop_line_y 최댓값 (-1 = 안 보임)
      roi   정면 1.3~1.6 m, 좌우 ±0.4 m 안 라이다 점 수 (front_1019 obs_evade 감지 구역)
      near  정면 ±0.4 m 안 가장 가까운 점 거리 m (9 = 없음)
라이다는 차 기준 180° 돌아 달려 있음: 차 앞 = 라이다 -x, 차 오른쪽 = +y (obs_evade와 같음).
"""
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROI_NEAR, ROI_FAR, ROI_HALF_W = 1.3, 1.6, 0.4
SKIP = {'/cam_front/image_raw/compressed', '/cam_lane/image_raw', '/lane_lines', '/lane_slope',
        '/lane_line_len', '/rosout'}
EVENTS = ('/drive_mode', '/lane_change_flag', '/lane_changing', '/crosswalk_active')


def main():
    cur = sqlite3.connect(sys.argv[1]).cursor()
    tp = {i: (n, get_message(t)) for i, n, t in cur.execute('select id,name,type from topics')}
    t0 = cur.execute('select min(timestamp) from messages').fetchone()[0]
    ev, last, bins = [], {}, {}
    for tid, ts, blob in cur.execute('select topic_id,timestamp,data from messages order by timestamp'):
        n, T = tp[tid]
        if n in SKIP:
            continue
        t = (ts - t0) / 1e9
        m = deserialize_message(blob, T)
        d = bins.setdefault(round(t * 2) / 2, {})
        if n in EVENTS:
            if last.get(n) != m.data:
                ev.append((t, f'{n} -> {m.data}'))
                last[n] = m.data
        elif n in ('/cmd_out', '/cmd_lane', '/cmd_obs', '/cmd_auto'):
            d[n] = (m.linear.x, m.angular.z)
        elif n in ('/light_stop', '/light_green'):
            d.setdefault(n, []).append(m.data)
        elif n == '/stop_line_y':
            if m.data >= 0:
                d['sl'] = max(d.get('sl', -1), m.data)
        elif n == '/scan':
            r = np.array(m.ranges)
            a = np.linspace(m.angle_min, m.angle_max, len(r))
            ok = (r > 0.1) & (r < 5)
            ah, rt = -(r[ok] * np.cos(a[ok])), r[ok] * np.sin(a[ok])
            d['roi'] = max(d.get('roi', 0), int(np.sum((ah > ROI_NEAR) & (ah < ROI_FAR) & (np.abs(rt) < ROI_HALF_W))))
            w = ah[(ah > 0.3) & (ah < 3) & (np.abs(rt) < ROI_HALF_W)]
            d['near'] = min(d.get('near', 9), float(w.min()) if w.size else 9)

    print('== events')
    for t, e in ev:
        print(f'{t:6.1f}s {e}')
    f = lambda p: f'{p[0]:+.2f},{p[1]:+5.0f}' if p else '     -     '
    print('== timeline: out | lane | obs | red/yel | green | stop_line_y | roi | near')
    for b in sorted(bins):
        d = bins[b]
        ls, lg = d.get('/light_stop', []), d.get('/light_green', [])
        print(f"{b:5.1f} out {f(d.get('/cmd_out'))} | lane {f(d.get('/cmd_lane'))} | obs {f(d.get('/cmd_obs'))} | "
              f"R {sum(ls)}/{len(ls)} G {sum(lg)}/{len(lg)} | sl {d.get('sl', -1):4.0f} | "
              f"roi {d.get('roi', 0):3d} | near {d.get('near', 9):4.2f}")


if __name__ == '__main__':
    main()
