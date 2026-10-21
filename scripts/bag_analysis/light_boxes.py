#!/usr/bin/env python3
"""녹화된 신호등 카메라(/cam_front/image_raw/compressed)에 sign_detect 모델을 다시 돌려 시간별
신호등 클래스·확신도·박스 넓이·위치를 출력. min_bbox_size(정지 위치)를 정할 때 씀(10/21: 10000).

사용:
    python3 scripts/bag_analysis/light_boxes.py ~/bags/obs_180551/*.db3 [간격초=0.5] [몽타주.jpg]

몽타주 경로를 주면 12장을 뽑아 박스를 그린 사진 한 장으로 저장(분석 뒤 지울 것 — 디스크 작음).
sign_detect 기준: 확신도 > light_conf(0.6), 넓이 >= min_bbox_size. 정지 중엔 x hold_bbox_ratio(0.7).
"""
import os
import sqlite3
import sys

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

MODEL = os.path.join(os.path.dirname(__file__), '..', '..', 'sign_detect', 'best_0808.pt')


def main():
    db = sys.argv[1]
    step = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    out = sys.argv[3] if len(sys.argv) > 3 else None
    cur = sqlite3.connect(db).cursor()
    r = cur.execute("select id from topics where name='/cam_front/image_raw/compressed'").fetchone()
    if not r:
        print('/cam_front/image_raw/compressed가 녹화되지 않음')
        return
    t0 = cur.execute('select min(timestamp) from messages').fetchone()[0]
    rows = cur.execute('select timestamp,data from messages where topic_id=? order by timestamp', (r[0],)).fetchall()
    m = YOLO(MODEL)
    last, ims = -1.0, []
    pick = set(np.linspace(0, len(rows) - 1, 12).astype(int)) if out else set()
    for i, (ts, blob) in enumerate(rows):
        t = (ts - t0) / 1e9
        if t - last < step and i not in pick:
            continue
        img = cv2.imdecode(np.frombuffer(bytes(deserialize_message(blob, CompressedImage).data), np.uint8), 1)
        res = m(img, verbose=False, conf=0.3)[0]
        if t - last >= step:
            last = t
            s = []
            for b in res.boxes:
                name = m.names[int(b.cls)]
                if 'light' not in name:
                    continue
                x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
                s.append(f"{name.replace('traffic-light-', '')} {float(b.conf):.2f} 넓이 {(x2 - x1) * (y2 - y1)} @({x1},{y1})")
            print(f'{t:6.2f}s', s or '-')
        if i in pick:
            p = res.plot()
            cv2.putText(p, f'{t:.1f}s', (5, 30), 0, 1, (0, 0, 255), 2)
            ims.append(cv2.resize(p, (320, 240)))
    if out and ims:
        while len(ims) % 4:
            ims.append(np.zeros_like(ims[0]))
        cv2.imwrite(out, np.vstack([np.hstack(ims[i:i + 4]) for i in range(0, len(ims), 4)]))
        print('saved', out)


if __name__ == '__main__':
    main()
