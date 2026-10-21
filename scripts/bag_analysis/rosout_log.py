#!/usr/bin/env python3
"""rosbag의 /rosout에서 obs_evade·decision_auto·sign_detect 로그만 뽑아 시간순으로 출력.

사용:
    python3 scripts/bag_analysis/rosout_log.py ~/bags/obs_180128/*.db3

/rosout은 녹화기에 늦게 도착하는 경우가 있어(10/21: 8초 늦음) 녹화 시각이 아니라 로그 자체의
시각(stamp)을 씀. 시각은 녹화 시작 기준 초. 차선 변경 단계가 "화면 조건 맞음"/"시간 초과" 중
무엇으로 끝났는지, 복귀 판단("Right side clear"), 신호등 정지("STOP SIGNAL ACTIVE")를 볼 때 씀.
녹화가 많으면 로그 일부가 빠질 수 있음(10/21 obs_175637: 앞 3개만 남음).
"""
import sqlite3
import sys

from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message

NODES = ('obs_evade', 'decision_auto', 'sign_detect')
NOISY = ('PRIORITY', 'STALE')   # 10Hz로 계속 찍히는 로그는 뺌


def main():
    cur = sqlite3.connect(sys.argv[1]).cursor()
    t0 = cur.execute('select min(timestamp) from messages').fetchone()[0]
    r = cur.execute("select id from topics where name='/rosout'").fetchone()
    if not r:
        print('/rosout이 녹화되지 않음')
        return
    for _, blob in cur.execute('select timestamp,data from messages where topic_id=? order by timestamp', (r[0],)):
        m = deserialize_message(blob, Log)
        if m.name in NODES and not any(k in m.msg for k in NOISY):
            st = m.stamp.sec * 1e9 + m.stamp.nanosec
            print(f'{(st - t0) / 1e9:6.2f}s [{m.name}] {m.msg}')


if __name__ == '__main__':
    main()
