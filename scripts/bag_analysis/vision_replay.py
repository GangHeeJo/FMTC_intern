#!/usr/bin/env python3
"""녹화된 /lane_lines로 obs_evade vision 차선 변경의 화면 조건을 다시 계산해서, 각 단계가 어떤 선을
보고 끝났는지(또는 왜 시간 초과였는지) 확인. /rosout 로그가 빠졌을 때 씀(10/21 obs_175637).

사용:
    python3 scripts/bag_analysis/vision_replay.py ~/bags/obs_175637/*.db3 [시작초] [끝초]

0.3초마다: 그때 /cmd_obs 조향(+1000 왼쪽 최대, -1000 오른쪽 최대, 0 직진) | 2→1 후보 | 1→2 후보 | 전체 선.
후보 = 길이 >= vis_len_min(250)이고 기울기가 vis_to1_k [-2.0, -0.3] / vis_to2_k [0.3, 2.0] 안인 선 중 가장 긴 것.
단계 끝 조건(obs_evade 기본값): 2→1 x >= -95, >= 180, <= 95 / 1→2 x <= 604, <= 378, >= 495,
마지막 단계는 |k| <= |k_end|(0.61 / 0.66)여도 끝. 값을 바꿨으면 아래 상수도 맞출 것.
"""
import sqlite3
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

LEN_MIN = 250.0
K1, K2 = (-2.0, -0.3), (0.3, 2.0)


def best(lines, k):
    c = [(l, x, kk) for x, kk, l in lines if k[0] <= kk <= k[1] and l >= LEN_MIN]
    if not c:
        return '        -          '
    l, x, kk = max(c)
    return f'x{x:5.0f} k{kk:+.2f} len{l:.0f}'


def main():
    db = sys.argv[1]
    t_from = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    t_to = float(sys.argv[3]) if len(sys.argv) > 3 else 1e9
    cur = sqlite3.connect(db).cursor()
    tp = {i: (n, get_message(t)) for i, n, t in cur.execute('select id,name,type from topics')}
    ids = {n: i for i, (n, _) in tp.items()}
    t0 = cur.execute('select min(timestamp) from messages').fetchone()[0]
    steer, lastp = 0.0, -1.0
    q = 'select topic_id,timestamp,data from messages where topic_id in (?,?) order by timestamp'
    for tid, ts, blob in cur.execute(q, (ids['/lane_lines'], ids.get('/cmd_obs', -1))):
        t = (ts - t0) / 1e9
        n, T = tp[tid]
        m = deserialize_message(blob, T)
        if n == '/cmd_obs':
            steer = m.angular.z
            continue
        if not (t_from <= t <= t_to) or t - lastp < 0.3:
            continue
        lastp = t
        d = list(m.data)
        lines = [(d[i], d[i + 1], d[i + 2]) for i in range(0, len(d) - 2, 3)]
        allv = ' '.join(f'({x:.0f},{k:+.2f},{l:.0f})' for x, k, l in lines)
        print(f'{t:5.1f}s steer {steer:+6.0f} | to1 {best(lines, K1)} | to2 {best(lines, K2)} | {allv[:100]}')


if __name__ == '__main__':
    main()
