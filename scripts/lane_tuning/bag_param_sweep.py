#!/usr/bin/env python3
"""저장된 rosbag의 차선 카메라 영상에 lane_detect를 '파라미터만 바꿔서' 다시 돌려 비교한다.
현장에서 차를 다시 굴리기 전에 후보 설정을 영상으로 먼저 걸러내는 용도.

사용 (워크스페이스 source 후):
    # min_slope를 세 값으로, bag의 6~36초 구간에서 비교
    python3 scripts/lane_tuning/bag_param_sweep.py --bag /tmp/bag5 --lo 6 --hi 36 \
        --param min_slope 0.4 0.35 0.3 --fixed guide_x=531 guide_slope=0.524
    # 여러 bag을 합쳐서(--bag 반복)
    python3 scripts/lane_tuning/bag_param_sweep.py --bag /tmp/bag5 --bag /tmp/bag9 --param roi_top 220 190

--param 으로 바꿀 수 있는 이름은 lane_detect의 인자: guide_x guide_slope a_rad b_rad min_slope right1 right2 roi_top

출력:
    검출%     : 선이 검출된 프레임 비율 (높을수록 좋음, 단 엉뚱한 선이면 의미 없음)
    포화%     : |z|>=900(조향 최대 근처) 비율 (낮을수록 좋음)
    변화량    : 5프레임 중앙값 필터 후 프레임 간 조향 변화 평균 (낮을수록 부드러움)
세 숫자를 같이 보고 판단할 것. 검출%만 높이면 엉뚱한 선(주차 박스 등)을 잡아 포화%/변화량이 나빠진다.
"""
import argparse
import glob
import sqlite3
import warnings

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import lane_trace.lane_trace as lt

warnings.filterwarnings("ignore")


def load_frames(bag, lo, hi, step):
    con = sqlite3.connect(glob.glob(bag + "/*.db3")[0])
    cur = con.cursor()
    topics = {i: (n, t) for i, n, t in cur.execute("select id,name,type from topics")}
    tid = [i for i, (n, _t) in topics.items() if n == "/cam_lane/image_raw"][0]
    t0 = cur.execute("select min(timestamp) from messages").fetchone()[0]
    frames = []
    q = "select timestamp,data from messages where topic_id=? order by timestamp"
    for k, (ts, blob) in enumerate(cur.execute(q, (tid,))):
        t = (ts - t0) / 1e9
        if k % step or not (lo <= t <= hi):
            continue
        m = deserialize_message(blob, get_message(topics[tid][1]))
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)
        if m.encoding == "rgb8":
            a = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
        frames.append(a)
    return frames


def parse_kv(items):
    out = {}
    for it in items or []:
        k, v = it.split("=")
        out[k] = float(v) if "." in v else int(v)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", action="append", required=True)
    ap.add_argument("--lo", type=float, default=0.0)
    ap.add_argument("--hi", type=float, default=1e9)
    ap.add_argument("--step", type=int, default=3, help="N프레임마다 1장 사용(속도)")
    ap.add_argument("--param", nargs="+", required=True, help="이름 값1 값2 ...")
    ap.add_argument("--fixed", nargs="*", help="고정할 값 이름=값 (예: guide_x=531)")
    a = ap.parse_args()

    frames = []
    for b in a.bag:
        frames += load_frames(b, a.lo, a.hi, a.step)
    fixed = parse_kv(a.fixed)
    name, values = a.param[0], [float(v) if "." in v else int(v) for v in a.param[1:]]
    print(f"프레임 {len(frames)}장, 고정 {fixed}")
    print(f"{name:12s}| 검출%  | 포화%  | 변화량")
    for v in values:
        kw = dict(fixed)
        kw[name] = v
        zs = []
        for im in frames:
            s = lt.lane_detect(im, **kw)[2]
            zs.append(None if s is None else -float(s) * 2500)
        det = np.array([z for z in zs if z is not None])
        if len(det):
            f = np.array([np.median(det[max(0, i - 4):i + 1]) for i in range(len(det))])
            d = np.abs(np.diff(f)) if len(f) > 1 else np.array([0.0])
            sat = 100 * np.mean(np.abs(det) >= 900)
            print(f"{v:<12}| {100 * len(det) // len(zs):3d}%   | {sat:5.1f}% | {d.mean():6.1f}")
        else:
            print(f"{v:<12}|   0%   |   -    |   -")


if __name__ == "__main__":
    main()
