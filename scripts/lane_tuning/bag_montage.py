#!/usr/bin/env python3
"""rosbag 영상에서 원하는 시각의 프레임을 뽑아 lane_detect 결과(overlay)를 2열 몽타주 이미지로 저장한다.
조향이 튀거나 차선을 놓친 시각의 장면을 눈으로 확인할 때 쓴다.

사용 (워크스페이스 source 후):
    python3 scripts/lane_tuning/bag_montage.py /tmp/bag9 /tmp/m.png 8 22 31 34
    python3 scripts/lane_tuning/bag_montage.py /tmp/bag9 /tmp/m.png 8 22 --kw min_slope=0.35 guide_x=531 guide_slope=0.524

화면의 빨간 세로선 두 개(x=330, x=640)가 인식 구역 경계이고, 빨간 대각선 아래쪽 점은 제외된다.
초록 선 = 후보로 잡힌 선, 파란 선 = 후보들을 피팅한 결과. 'NO LANE' = 검출 실패.
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("times", nargs="+", type=float)
    ap.add_argument("--kw", nargs="*", help="lane_detect 인자 이름=값")
    a = ap.parse_args()
    kw = {}
    for it in a.kw or []:
        k, v = it.split("=")
        kw[k] = float(v) if "." in v else int(v)

    con = sqlite3.connect(glob.glob(a.bag + "/*.db3")[0])
    cur = con.cursor()
    topics = {i: (n, t) for i, n, t in cur.execute("select id,name,type from topics")}
    tid = [i for i, (n, _t) in topics.items() if n == "/cam_lane/image_raw"][0]
    t0 = cur.execute("select min(timestamp) from messages").fetchone()[0]
    imgs = {}
    for ts, blob in cur.execute("select timestamp,data from messages where topic_id=? order by timestamp", (tid,)):
        t = (ts - t0) / 1e9
        for w in a.times:
            if w not in imgs and t >= w:
                m = deserialize_message(blob, get_message(topics[tid][1]))
                img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)
                if m.encoding == "rgb8":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                if img.shape[1] != 640 or img.shape[0] != 480:  # lane_trace와 같게 640x480 좌표로 맞춤(320x240 bag)
                    img = cv2.resize(img, (640, 480), interpolation=cv2.INTER_LINEAR)
                ov, _mask, steer = lt.lane_detect(img, **kw)
                label = f"t={w:.0f}s " + ("NO LANE" if steer is None else f"z={-steer * 2500:+.0f}")
                cv2.putText(ov, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                imgs[w] = cv2.resize(ov, (480, 360))
    tiles = [imgs[w] for w in a.times if w in imgs]
    while len(tiles) % 2:
        tiles.append(np.zeros_like(tiles[0]))
    cv2.imwrite(a.out, np.vstack([np.hstack(tiles[i:i + 2]) for i in range(0, len(tiles), 2)]))
    print("저장:", a.out, f"({len(imgs)}장)")


if __name__ == "__main__":
    main()
