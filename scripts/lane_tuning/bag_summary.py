#!/usr/bin/env python3
"""rosbag(sqlite3)에서 제어 체인을 2초 단위로 요약한다.

기록 (워크스페이스 source 후):
    ros2 bag record -o /tmp/bagX /cmd_lane /cmd_auto /cmd_out /light_stop /cross_stop /drive_mode /joy /cam_lane/image_raw
    (영상 없이 가볍게 하려면 마지막 토픽 제외)
    # 기록 전에 중복 노드가 없는지: ros2 node list | sort | uniq -c   (모두 1이어야 함)

분석:
    python3 scripts/lane_tuning/bag_summary.py /tmp/bagX

출력 열:
    lane 전진%  : lane_trace가 전진(linear.x != 0)을 낸 비율. 낮으면 차선을 놓치는 중
    out 전진%   : 최종적으로 모터로 나간 전진 비율. lane보다 많이 낮으면 cmd_mux/decision_auto가 막는 중
    out 조향    : 모터로 나간 angular.z(-1000..1000). +는 왼쪽, -는 오른쪽(관측 기준)
    |z|>=900%   : 조향이 최대 근처로 몰린 비율. 높으면 포화(한쪽 끝까지 꺾임)
"""
import glob
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    d = sys.argv[1]
    con = sqlite3.connect(glob.glob(d + "/*.db3")[0])
    cur = con.cursor()
    topics = {i: (n, t) for i, n, t in cur.execute("select id,name,type from topics")}
    t0 = cur.execute("select min(timestamp) from messages").fetchone()[0]
    data, nimg = {}, 0
    for tid, ts, blob in cur.execute("select topic_id,timestamp,data from messages order by timestamp"):
        name, typ = topics[tid]
        t = (ts - t0) / 1e9
        if name == "/cam_lane/image_raw":
            nimg += 1
            continue
        m = deserialize_message(blob, get_message(typ))
        if name in ("/cmd_out", "/cmd_lane", "/cmd_auto"):
            data.setdefault(name, []).append((t, m.linear.x, m.angular.z))
        elif name == "/drive_mode":
            data.setdefault(name, []).append((t, m.data))
        elif name == "/joy":
            data.setdefault(name, []).append((t, list(m.buttons)))
        elif name in ("/light_stop", "/cross_stop"):
            data.setdefault(name, []).append((t, m.data))
    print({k: len(v) for k, v in data.items()}, "영상", nimg)
    prev = None
    for t, b in data.get("/joy", []):
        if prev:
            for i in (0, 1, 3):  # A=자동, B=수동, Y=정지
                if b[i] and not prev[i]:
                    print(f"  버튼 {'ABXY'[i]} {t:5.1f}s")
        prev = b
    print("  모드 변화(0수동 1자동 2정지):", [(round(t, 1), v) for t, v in data.get("/drive_mode", [])])
    print("  정지신호 True:", sum(1 for _, v in data.get("/light_stop", []) + data.get("/cross_stop", []) if v), "개")
    if "/cmd_out" not in data:
        print("/cmd_out이 없습니다(모드가 STOP이었거나 노드가 안 떠 있었음).")
        return
    T = max(t for t, *_ in data["/cmd_out"])
    print("구간(s) | lane 전진% | out 전진% | out 조향 평균 [최소~최대] | |z|>=900%")
    for b in np.arange(0, T, 2.0):
        lane = [x for x in data.get("/cmd_lane", []) if b <= x[0] < b + 2]
        out = [x for x in data["/cmd_out"] if b <= x[0] < b + 2]
        if not out:
            continue
        z = [x[2] for x in out]
        lf = 100 * sum(1 for x in lane if x[1] != 0) // max(1, len(lane)) if lane else -1
        of = 100 * sum(1 for x in out if x[1] != 0) // len(out)
        sat = 100 * sum(1 for v in z if abs(v) >= 900) // len(z)
        print(f"{b:4.0f}-{b + 2:3.0f} |   {lf:3d}%    |   {of:3d}%   | {np.mean(z):+6.0f} [{min(z):+5.0f}~{max(z):+5.0f}] | {sat:3d}%")


if __name__ == "__main__":
    main()
