#!/usr/bin/env python3
"""녹화된 차선 카메라(/cam_lane/image_raw)에서 지정한 시각의 프레임을 뽑아 한 장으로 붙임.
차가 실제로 어디를 향하는지(복귀 과조향, 노트북 화면이 시야를 가리는지 등) 볼 때 씀(10/21).

사용:
    python3 scripts/bag_analysis/lane_frames.py ~/bags/obs_175637/*.db3 out.jpg 11 13.4 16.5 18.6 ...

시각은 녹화 시작 기준 초, 4장씩 한 줄. 원본 영상은 1분에 약 420 MB이니 분석 뒤 bag·사진은 지울 것.
"""
import sqlite3
import sys

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


def main():
    db, out, want = sys.argv[1], sys.argv[2], [float(x) for x in sys.argv[3:]]
    cur = sqlite3.connect(db).cursor()
    r = cur.execute("select id from topics where name='/cam_lane/image_raw'").fetchone()
    if not r:
        print('/cam_lane/image_raw가 녹화되지 않음')
        return
    t0 = cur.execute('select min(timestamp) from messages').fetchone()[0]
    rows = cur.execute('select timestamp,data from messages where topic_id=? order by timestamp', (r[0],)).fetchall()
    br, ims = CvBridge(), []
    for w in want:
        ts, blob = min(rows, key=lambda x: abs((x[0] - t0) / 1e9 - w))
        img = cv2.resize(br.imgmsg_to_cv2(deserialize_message(blob, Image), 'bgr8'), (320, 240))
        cv2.putText(img, f'{(ts - t0) / 1e9:.1f}s', (5, 20), 0, 0.6, (0, 0, 255), 2)
        ims.append(img)
    while len(ims) % 4:
        ims.append(np.zeros_like(ims[0]))
    cv2.imwrite(out, np.vstack([np.hstack(ims[i:i + 4]) for i in range(0, len(ims), 4)]))
    print('saved', out)


if __name__ == '__main__':
    main()
