#!/usr/bin/env python3
"""차를 '원하는 위치와 방향'에 놓고 실행하면, 그 자세에서 선의 기울기/위치를 측정해
lane_trace의 guide_slope / guide_x 값을 제안한다.

사용 (s1_stack과 usb_cam이 떠 있는 상태, 워크스페이스를 source한 터미널):
    python3 scripts/lane_tuning/calib_pose.py            # 5초 측정
    python3 scripts/lane_tuning/calib_pose.py --sec 8 --save /tmp/calib.png

출력의 '제안'을 그대로 lane_trace 실행 명령에 붙인다:
    ros2 run lane_trace lane_trace --ros-args -p guide_slope:=<값> -p guide_x:=<값> ...
"""
import argparse
import time
import warnings

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

import lane_trace.lane_trace as lt

warnings.filterwarnings("ignore")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sec", type=float, default=5.0)
    ap.add_argument("--topic", default="/cam_lane/image_raw")
    ap.add_argument("--save", default="", help="마지막 프레임의 인식 결과(overlay)를 저장할 경로")
    ap.add_argument("--l_min", type=int, default=100, help="lane_trace의 l_min과 같게")
    ap.add_argument("--s_max", type=int, default=255, help="lane_trace의 s_max와 같게")
    a = ap.parse_args()

    rclpy.init()
    node = Node("calib_pose")
    bridge = CvBridge()
    rows, last = [], {}

    def cb(msg):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        if img.shape[1] != 640 or img.shape[0] != 480:  # lane_trace와 같게 640x480 좌표로 맞춤
            img = cv2.resize(img, (640, 480), interpolation=cv2.INTER_LINEAR)
        overlay, _mask, steer = lt.lane_detect(img, l_min=a.l_min, s_max=a.s_max)  # 기본 기준값으로 호출, 측정값(last_metrics)만 사용
        last["overlay"] = overlay
        rows.append((lt.last_metrics["slope_avg"], lt.last_metrics["x_value"])
                    if steer is not None and lt.last_metrics else None)

    node.create_subscription(Image, a.topic, cb, qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < a.sec:
        rclpy.spin_once(node, timeout_sec=0.05)

    ok = [r for r in rows if r]
    print(f"프레임 {len(rows)}개 중 차선 검출 {len(ok)}개 ({100 * len(ok) // max(1, len(rows))}%)")
    if ok:
        sl = np.array([r[0] for r in ok])
        xs = np.array([r[1] for r in ok])
        print(f"기울기(rad): 평균 {sl.mean():.3f}  표준편차 {sl.std():.3f}  범위 {sl.min():.3f}~{sl.max():.3f}")
        print(f"위치 x(px) : 평균 {xs.mean():.1f}  표준편차 {xs.std():.1f}  범위 {xs.min():.0f}~{xs.max():.0f}")
        print(f"제안: -p guide_slope:={np.median(sl):.3f} -p guide_x:={np.median(xs):.0f}")
        if sl.std() > 0.05 or xs.std() > 30:
            print("주의: 값이 많이 흔들립니다. 차를 고정하고 차선이 잘 보이는 자세로 다시 측정하세요.")
    else:
        print("차선이 한 번도 검출되지 않았습니다. 오른쪽 경계선이 카메라 화면 오른쪽에 보이도록 놓으세요.")
    if a.save and "overlay" in last:
        cv2.imwrite(a.save, last["overlay"])
        print("저장:", a.save)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
