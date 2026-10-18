#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import time
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

def remove_thin(mask, open_k):
    # open_k px보다 가는 흰 선(주차 칸 선 등)을 지움. 차선(실선·점선)은 굵어서 남음. 1 이하면 끔
    if open_k <= 1:
        return mask
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_k, open_k)))

def remove_big(mask, max_area):
    # max_area(px², 640x480 기준)보다 큰 흰 덩어리를 지움. 점선 한 토막은 4천~8천, 옆 차선의 흰 장애물 차체는
    # 1만1천~5만이라 1차선 직후 차체 옆선을 선으로 잡는 것 방지(10/17). 0 이하면 끔
    if max_area <= 0:
        return mask
    n, lab, st, _ = cv2.connectedComponentsWithStats(mask)
    big = [i for i in range(1, n) if st[i, cv2.CC_STAT_AREA] > max_area]
    if big:
        mask = mask.copy()
        mask[np.isin(lab, big)] = 0
    return mask

def color_filter(image, l_min=100, s_max=255):
    # 햇빛 변화에 강한 HLS 색 공간 변환
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

    # 흰색 차선 필터링 (조도에 따라 L값 범위를 조절하세요)
    # 흰 선은 채도(S)가 낮음(0~7). 녹색 잔디는 L이 흰 선과 비슷해도 S가 35~40이라 s_max로 걸러짐
    lower_white = np.array([0, l_min, 0])      #150~200
    upper_white = np.array([255, 255, s_max])
    
    mask = cv2.inRange(hls, lower_white, upper_white)
    return mask

def image_crop(image, start_point=(100, 220), end_point=(640, 450)):
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.rectangle(mask, start_point, end_point, 255, thickness=cv2.FILLED)
    return cv2.bitwise_and(image, image, mask=mask)

def get_x_coordinate(y_value, slope, y_intercept):
    if slope == 0: return 320
    return int((y_value - y_intercept) / slope)

def auto_canny(image, sigma=0.33):
    # 가우시안 블러로 노이즈 제거
    image_blur = cv2.GaussianBlur(image, (5, 5), 0)
    v = np.median(image_blur)
    
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image_blur, lower, upper) # 블러 처리된 이미지를 사용
    return edged

# 마지막 검출의 선 기울기/위치 (기준값 보정용으로 외부에서 읽음)
last_metrics = {}


# 우리 차 보닛 영역(640x480). 10/17 bag 650장에서 70% 이상 흰색인 부분의 윗선 + 아래 모서리
HOOD_POLY = np.array([[0, 280], [80, 252], [144, 242], [191, 244], [251, 262], [292, 292],
                      [452, 479], [0, 479]], np.int32)


def longest_line_len(image, l_min, s_max, open_k, roi_top=100):
    """화면에서 차선으로 보이는 흰 덩어리 중 가장 긴 것의 길이(px, 640x480). 없으면 0.
    2차선 실선은 화면을 가로질러 633~742, 1차선 점선은 토막이라 94~346(10/17 bag) → 450 기준으로 차선 구분"""
    mask = remove_thin(color_filter(image, l_min, s_max), open_k)
    cv2.fillPoly(mask, [HOOD_POLY], 0)
    mask[:roi_top, :] = 0
    n, lab, st, _ = cv2.connectedComponentsWithStats(mask)
    dt = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    best = 0.0
    for i in range(1, n):
        if not (300 <= st[i, cv2.CC_STAT_AREA] <= 40000):
            continue
        ys, xs = np.nonzero(lab == i)
        d = dt[ys, xs]
        if not (10.0 <= 2.0 * float(np.median(d[d >= np.percentile(d, 80)])) <= 40.0):
            continue
        vx, vy, x0, y0 = cv2.fitLine(np.column_stack([xs, ys]).astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01).ravel()
        if vx < 0:
            vx, vy = -vx, -vy
        if vy / max(vx, 1e-6) < 0.1:
            continue
        proj = (xs - x0) * vx + (ys - y0) * vy
        best = max(best, float(proj.max() - proj.min()))
    return best


def lane_detect_v2(image, guide_x, guide_slope, a_rad, b_rad, min_slope, roi_top, l_min, s_max, open_k,
                   max_blob, min_thick=10.0, max_thick=40.0, min_elong=2.5, x_min=150, x_max=750, join_px=40,
                   max_area=20000, short_len=300, min_fill=0.75, min_solid=0.85):
    """흰 덩어리를 모양으로 나눠 차선만 고름(10/17).
    주차선: 가늘다(open_k로 지움) / 흰 차: 굵다(max_thick), 조각은 모양이 들쭉날쭉(fill, solid) /
    점선: 굵기 20~33px, 꽉 찬 직사각형(fill 0.9~1.0, solid 0.96~) / 실선: 길게 이어짐(short_len 넘으면 모양 검사 안 함).
    (10/17 bag 측정값. max_blob은 쓰지 않음: 가까운 점선 토막이 1만2천까지 커짐)
    보닛은 HOOD_POLY로 지워서 기존의 빨간 대각선 제외 없이 가까운 점선 토막도 씀.
    후보 중 y=300 위치가 guide_x에 가장 가까운 선을 고르고, 그 선과 한 줄에 있는 토막(점선)을 모아 다시 맞춤."""
    overlay = image.copy()
    mask = remove_thin(color_filter(image, l_min, s_max), open_k)
    cv2.fillPoly(mask, [HOOD_POLY], 0)
    mask[:roi_top, :] = 0
    n, lab, st, _ = cv2.connectedComponentsWithStats(mask)
    dt = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    cands = []
    for i in range(1, n):
        area = st[i, cv2.CC_STAT_AREA]
        if area < 300 or area > max_area:
            continue
        ys, xs = np.nonzero(lab == i)
        d = dt[ys, xs]
        thick = 2.0 * float(np.median(d[d >= np.percentile(d, 80)]))
        if not (min_thick <= thick <= max_thick):
            continue
        pts = np.column_stack([xs, ys]).astype(np.float32)
        vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).ravel()
        if vx < 0:
            vx, vy = -vx, -vy
        proj = (xs - x0) * vx + (ys - y0) * vy
        length = float(proj.max() - proj.min())
        if length < min_elong * thick or abs(vy) < 1e-3 or abs(vx) < 1e-3:
            continue
        if length < short_len:
            # 짧은 조각은 꽉 찬 직사각형이어야 함(점선 토막). 흰 차 조각은 들쭉날쭉함
            w95 = 2.0 * float(np.percentile(np.abs(-(xs - x0) * vy + (ys - y0) * vx), 95))
            hull_area = cv2.contourArea(cv2.convexHull(pts.astype(np.int32)))
            if area / max(1.0, length * w95) < min_fill or area / max(1.0, hull_area) < min_solid:
                continue
        slope = vy / vx  # 화면 dy/dx. 차선은 왼쪽 위→오른쪽 아래라 양수
        if slope < min_slope:
            continue
        x300 = x0 + (300.0 - y0) * vx / vy
        if not (x_min <= x300 <= x_max):
            continue
        cands.append(dict(i=i, xs=xs, ys=ys, x300=float(x300), slope=float(slope)))
    for c in cands:
        overlay[c['ys'], c['xs']] = (0, 200, 0)
    if not cands:
        last_metrics.clear()
        return overlay, mask, None
    best = min(cands, key=lambda c: abs(c['x300'] - guide_x))
    group = [c for c in cands if abs(c['x300'] - best['x300']) < join_px
             and abs(np.arctan(c['slope']) - np.arctan(best['slope'])) < 0.25]
    xs = np.concatenate([c['xs'] for c in group]); ys = np.concatenate([c['ys'] for c in group])
    vx, vy, x0, y0 = cv2.fitLine(np.column_stack([xs, ys]).astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01).ravel()
    slope_avg = abs(float(np.arctan2(vy, vx)))
    if slope_avg > np.pi / 2:
        slope_avg = np.pi - slope_avg
    x_value = float(x0 + (300.0 - y0) * vx / vy)
    y_lo, y_hi = int(ys.min()), int(ys.max())
    cv2.line(overlay, (int(x0 + (y_lo - y0) * vx / vy), y_lo), (int(x0 + (y_hi - y0) * vx / vy), y_hi), (255, 0, 0), 3)
    cv2.polylines(overlay, [HOOD_POLY.reshape(-1, 1, 2)], True, (0, 0, 255), 1)
    last_metrics.update({'slope_avg': slope_avg, 'x_value': x_value, 'n_parts': len(group)})
    steering_rad = a_rad * (slope_avg - guide_slope) + b_rad * (x_value - guide_x)
    return overlay, mask, float(np.clip(steering_rad, -0.4, 0.4))


def lane_detect(image, guide_x=550, guide_slope=1.11, a_rad=0.5, b_rad=0.0027,
                min_slope=0.4, right1=330, right2=640, roi_top=220, l_min=100, s_max=255, open_k=1, max_blob=0,
                detector=1, **v2):
    if detector == 2:
        return lane_detect_v2(image, guide_x, guide_slope, a_rad, b_rad, min_slope, roi_top, l_min, s_max,
                              open_k, max_blob, **v2)

    result_overlay = image.copy()
    mask = remove_big(remove_thin(color_filter(image, l_min, s_max), open_k), max_blob)

    # 기존 ROI 유지
    start_point = (100, roi_top)
    end_point = (640, 450)
    mask_crop = image_crop(mask, start_point, end_point)
    
    # color_filter -> auto canny
    edge_crop = auto_canny(mask_crop, sigma=0.33)


    lines = cv2.HoughLinesP(edge_crop, rho=1, theta=np.pi/180, threshold=20, minLineLength=65, maxLineGap=10) # 70 -> 65
    
    filtered_points = []
    
    # 관심 영역 상수 (기존 유지)
    m, n, k = 0.3, (620, 380), 700
    
    # 필터링용 사선 및 수직선 (빨간색)
    cross_x = n[0] - k
    cross_y = int(n[1] - k * m)
    
    cv2.line(result_overlay, n, (cross_x, cross_y), (0, 0, 255), 1)
    cv2.line(result_overlay, (right1, 0), (right1, 480), (0, 0, 255), 1)
    cv2.line(result_overlay, (right2, 0), (right2, 480), (0, 0, 255), 1)

    # 가이드 및 조향 상수는 함수 인자로 받음 (기본값은 기존 튜닝값)
    # a_rad: 각도 오차 가중치, b_rad: 위치 오차(픽셀) 가중치

    slope_avg, i, sum_slope = guide_slope, 0, 0

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else 999.0
            midpoint = ((x1 + x2) / 2, (y1 + y2) / 2)
            
            # ROI 필터링: m_2(우측 상단 대각선) 조건을 제거하여 영역을 확장함
            if ((midpoint[0] > right1) and (midpoint[0] < right2) and 
                (m * (midpoint[0] - n[0]) + (n[1] - midpoint[1]) > 0) and 
                abs(slope) > min_slope):
                filtered_points.append((x1, y1))
                filtered_points.append((x2, y2))
                cv2.line(result_overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
                i += 1
                sum_slope += slope

    # 4. 최종 계산
    x_value = guide_x
    
    # 검출 여부를 확인하기 위한 플래그
    detected = False
    
    if i > 0:
        slope_avg = abs(np.arctan(sum_slope / i))
        detected = True # 차선 검출됨

    if len(filtered_points) > 0:
        points_arr = np.array(filtered_points)
        slope_fit, y_intercept_fit = np.polyfit(points_arr[:, 0], points_arr[:, 1], deg=1)
        x_value = get_x_coordinate(300, slope_fit, y_intercept_fit)
        
        x1_f, x2_f = int(min(points_arr[:, 0])), int(max(points_arr[:, 0]))
        cv2.line(result_overlay, (x1_f, int(slope_fit * x1_f + y_intercept_fit)), 
                 (x2_f, int(slope_fit * x2_f + y_intercept_fit)), (255, 0, 0), 3)

    # 검출 실패 시 None 반환
    if not detected:
        last_metrics.clear()
        return result_overlay, mask, None

    last_metrics.update({'slope_avg': float(slope_avg), 'x_value': float(x_value)})

    # 5. 조향값 직접 계산 (이미 rad 단위이므로 Scaling 불필요)
    steering_rad = a_rad * (slope_avg - guide_slope) + b_rad * (x_value - guide_x)
    
    # 바로 0.4 rad로 클램핑
    steering_rad = np.clip(steering_rad, -0.4, 0.4)

    return result_overlay, mask, steering_rad


class LaneMaskingNode(Node):
    def __init__(self):
        super().__init__('lane_masking_node')
        self.cv_bridge = CvBridge()

        self.declare_parameter('debug_view', False)
        self.debug_view = self.get_parameter('debug_view').get_parameter_value().bool_value

        # 차선 추종 시 전진 속도(linear.x, 0~1). serial_sender의 th_scale과 곱해져 실제 TH가 됨
        self.declare_parameter('linear_speed', 0.5, ParameterDescriptor(dynamic_typing=True))
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        # 차선을 놓친 직후 이 시간(초) 동안은 마지막 조향으로 계속 전진. 0이면 놓치는 즉시 정지(기존 동작)
        self.declare_parameter('lost_hold_s', 0.0, ParameterDescriptor(dynamic_typing=True))
        self.lost_hold_s = float(self.get_parameter('lost_hold_s').value)
        self.last_detect_t = None
        # 최근 detect_window_s초 동안 검출된 프레임 비율이 min_detect_ratio 미만이면 정지.
        # 띄엄띄엄 잡히는 검출이 lost_hold_s 타이머를 계속 갱신해 차선을 거의 놓친 채 달리는 것을 막음. 0이면 끔(기존 동작)
        self.declare_parameter('min_detect_ratio', 0.0, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('detect_window_s', 0.5, ParameterDescriptor(dynamic_typing=True))
        self.min_detect_ratio = float(self.get_parameter('min_detect_ratio').value)
        self.detect_window_s = float(self.get_parameter('detect_window_s').value)
        self.detect_hist = []  # (시각, 검출 여부)

        # 조향 기준값: 차를 원하는 위치/방향에 놓았을 때 측정한 선의 위치(px)와 기울기(rad)
        self.declare_parameter('guide_x', 550.0, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('guide_slope', 1.11, ParameterDescriptor(dynamic_typing=True))
        self.guide_x = float(self.get_parameter('guide_x').value)
        self.guide_slope = float(self.get_parameter('guide_slope').value)
        # 1차선 주행(lane_change_flag=True) 때의 기준값: 오른쪽 가운데 점선을 따라감. 안 주면 2차선 값과 같음
        self.declare_parameter('guide_x_l1', self.guide_x, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('guide_slope_l1', self.guide_slope, ParameterDescriptor(dynamic_typing=True))
        self.guide_x_l1 = float(self.get_parameter('guide_x_l1').value)
        self.guide_slope_l1 = float(self.get_parameter('guide_slope_l1').value)
        # True면 예전 동작: 1차선일 때 영상과 조향을 좌우 반전. 차선 카메라가 오른쪽을 봐서 1차선에서도
        # 따라갈 선(점선)이 오른쪽에 있으므로 기본은 끔
        self.declare_parameter('flip_on_lane_change', False)
        self.flip_on_lane_change = bool(self.get_parameter('flip_on_lane_change').value)
        # 인식 구역 오른쪽 경계(px). 1차선에서는 점선 오른쪽의 주차 구역 선이 같이 잡히므로 좁힘(10/16: 440)
        self.declare_parameter('right2', 640, ParameterDescriptor(dynamic_typing=True))
        self.right2 = int(self.get_parameter('right2').value)
        self.declare_parameter('right2_l1', self.right2, ParameterDescriptor(dynamic_typing=True))
        self.right2_l1 = int(self.get_parameter('right2_l1').value)
        # 1차선 정지 조건: 점선은 칸 사이가 비어 검출이 끊기므로 창을 늘리고 기준을 낮출 수 있게 따로 둠
        self.declare_parameter('min_detect_ratio_l1', self.min_detect_ratio, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('detect_window_s_l1', self.detect_window_s, ParameterDescriptor(dynamic_typing=True))
        self.min_detect_ratio_l1 = float(self.get_parameter('min_detect_ratio_l1').value)
        self.detect_window_s_l1 = float(self.get_parameter('detect_window_s_l1').value)
        # 1차선에서 점선 칸 사이(선이 안 보이는 구간)를 마지막 조향으로 지나갈 시간(초)
        self.declare_parameter('lost_hold_s_l1', self.lost_hold_s, ParameterDescriptor(dynamic_typing=True))
        self.lost_hold_s_l1 = float(self.get_parameter('lost_hold_s_l1').value)
        # 1차선에서 선이 안 보이는 동안 유지하는 조향의 상한(도). 방향을 크게 바로잡던 중 선을 놓치면
        # 큰 조향으로 lost_hold_s_l1 동안 계속 도는 것을 막음(10/16: 5). 기본 23(=조향 최대치, 제한 없음)
        self.declare_parameter('hold_steer_max_deg_l1', 23.0, ParameterDescriptor(dynamic_typing=True))
        self.hold_steer_max_l1 = np.radians(float(self.get_parameter('hold_steer_max_deg_l1').value))

        # 차선 후보로 인정하는 조건: 선의 최소 기울기, 인식 구역 왼쪽 경계(px). 코너에서 휜 선을 잡으려면 낮춤
        self.declare_parameter('min_slope', 0.4, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('right1', 330, ParameterDescriptor(dynamic_typing=True))
        self.min_slope = float(self.get_parameter('min_slope').value)
        self.right1 = int(self.get_parameter('right1').value)

        # 인식 구역 위쪽 경계(px, 클수록 가까운 곳만 봄)와 조향 중앙값 필터 길이(1이면 끔)
        self.declare_parameter('roi_top', 220, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('steer_filter_n', 1, ParameterDescriptor(dynamic_typing=True))
        self.roi_top = int(self.get_parameter('roi_top').value)
        # 1차선 인식 구역 위쪽 경계(px). 차가 1차선 안쪽(왼쪽)으로 깊이 들어가면 점선이 화면 위쪽에 보임(10/16: 100)
        self.declare_parameter('roi_top_l1', self.roi_top, ParameterDescriptor(dynamic_typing=True))
        self.roi_top_l1 = int(self.get_parameter('roi_top_l1').value)
        # 차선 변경 중(/lane_changing=True)에만 쓰는 인식 조건. 꺾는 동안 새 차선의 선이 화면 위쪽에 눕게 보여
        # 기본 조건(roi_top, min_slope)으로는 늦게 잡힘(10/16: 1→2차선 복귀 때 못 잡음). -1이면 평소 값 그대로
        self.declare_parameter('roi_top_change', -1, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('min_slope_change', -1.0, ParameterDescriptor(dynamic_typing=True))
        self.roi_top_change = int(self.get_parameter('roi_top_change').value)
        self.min_slope_change = float(self.get_parameter('min_slope_change').value)
        # 2차선으로 돌아갈 때(1→2) 따로 쓰는 값. 안 주면 위와 같음. 0.1이면 멀리 누운 경계선을 일찍 잡아
        # 정면 복귀가 8초 안에 안 끝남(10/17 5차: 기울기 0.13에서 시작) → 조금 늦게 잡도록 올림
        self.declare_parameter('roi_top_change_l2', self.roi_top_change, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('min_slope_change_l2', self.min_slope_change, ParameterDescriptor(dynamic_typing=True))
        self.roi_top_change_l2 = int(self.get_parameter('roi_top_change_l2').value)
        self.min_slope_change_l2 = float(self.get_parameter('min_slope_change_l2').value)
        self.lane_changing = False
        self.steer_filter_n = max(1, int(self.get_parameter('steer_filter_n').value))
        self.steer_hist = []

        # 흰 선으로 인정하는 HLS 조건: 밝기 하한, 채도 상한. 기본값(100, 255)은 기존 동작(밝기만 봄)
        self.declare_parameter('l_min', 100, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('s_max', 255, ParameterDescriptor(dynamic_typing=True))
        self.l_min = int(self.get_parameter('l_min').value)
        self.s_max = int(self.get_parameter('s_max').value)
        # 이 크기(px)보다 가는 흰 선은 지움. 1차선 오른쪽 주차 칸 선을 점선으로 착각하는 것 방지(10/16: 9)
        self.declare_parameter('open_k', 1, ParameterDescriptor(dynamic_typing=True))
        self.open_k = int(self.get_parameter('open_k').value)
        # 이 면적(px²)보다 큰 흰 덩어리는 선이 아님(0이면 끔). 1차선 값은 따로: 1차선 직후 옆 장애물 차체 오인 방지
        self.declare_parameter('max_blob', 0, ParameterDescriptor(dynamic_typing=True))
        self.max_blob = int(self.get_parameter('max_blob').value)
        self.declare_parameter('max_blob_l1', self.max_blob, ParameterDescriptor(dynamic_typing=True))
        self.max_blob_l1 = int(self.get_parameter('max_blob_l1').value)
        # 선 검출 방식. 1 = 기존(Hough + 고정 구역), 2 = lane_detect_v2(흰 덩어리 모양으로 점선/실선/주차선/차 구분)
        # detector_l1은 1차선 주행 중(차선 변경 중 제외)에만 씀. 10/17 bag: 1차선 검출 40~60% → 98~100%
        self.declare_parameter('detector', 1, ParameterDescriptor(dynamic_typing=True))
        self.detector = int(self.get_parameter('detector').value)
        self.declare_parameter('detector_l1', self.detector, ParameterDescriptor(dynamic_typing=True))
        self.detector_l1 = int(self.get_parameter('detector_l1').value)
        # 차선 변경 중(/lane_changing)에 쓸 검출 방식. obs_evade의 시간 값들이 기존 방식(1) 기준으로 맞춰져 있어 기본 1
        self.declare_parameter('detector_change', 1, ParameterDescriptor(dynamic_typing=True))
        self.detector_change = int(self.get_parameter('detector_change').value)
        # 2차선을 방식 2로 따라갈 때의 기준값. 방식 2는 휜 실선 전체를 맞춰서 방식 1보다 위치 약 20px 왼쪽,
        # 기울기 0.03 가파르게 읽음 → 384/0.73이면 10/17 2차선 주행 bag에서 방식 1과 조향 중앙값이 같음(±70/1000)
        self.declare_parameter('guide_x_v2', float(self.guide_x), ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('guide_slope_v2', float(self.guide_slope), ParameterDescriptor(dynamic_typing=True))
        self.guide_x_v2 = float(self.get_parameter('guide_x_v2').value)
        self.guide_slope_v2 = float(self.get_parameter('guide_slope_v2').value)

        # 조향 게인: 기울기 오차(rad)와 위치 오차(px) 가중치. 커브에서 덜 꺾여 바깥으로 밀리면 b_rad를 올림
        self.declare_parameter('a_rad', 0.5, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('b_rad', 0.0027, ParameterDescriptor(dynamic_typing=True))
        self.a_rad = float(self.get_parameter('a_rad').value)
        self.b_rad = float(self.get_parameter('b_rad').value)
        # 1차선 조향 게인. 1차선은 커브 안쪽이라 더 꺾어야 함(10/16: 2차선 b 0.004로는 부족). 안 주면 2차선 값과 같음
        self.declare_parameter('a_rad_l1', self.a_rad, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('b_rad_l1', self.b_rad, ParameterDescriptor(dynamic_typing=True))
        self.a_rad_l1 = float(self.get_parameter('a_rad_l1').value)
        self.b_rad_l1 = float(self.get_parameter('b_rad_l1').value)

        # Subscriber & Publisher
        self.img_sub = self.create_subscription(
            Image, '/cam_lane/image_raw', self.image_callback, qos_profile_sensor_data)
        self.lane_change_sub = self.create_subscription(Bool, '/lane_change_flag', self.lane_change_callback, 10)
        self.lane_changing_sub = self.create_subscription(Bool, '/lane_changing', self.lane_changing_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_lane', 10)
        # 이번 프레임에서 선을 찾았는지 (obs_evade가 차선 변경 완료를 판단하는 데 씀)
        self.detected_pub = self.create_publisher(Bool, '/lane_detected', 10)
        # 이번 프레임에서 찾은 선의 기울기(rad, 못 찾으면 -1). obs_evade가 차선 변경 후 정면 복귀 판단에 씀
        self.slope_pub = self.create_publisher(Float32, '/lane_slope', 10)
        # 가장 긴 선의 길이(px). obs_evade가 출발 차선 자동 판단에 씀(실선 = 길다 → 2차선, 점선 = 짧다 → 1차선)
        self.line_len_pub = self.create_publisher(Float32, '/lane_line_len', 10)
        
        self.lane_change_flag = 0
        self.last_steer_rad = 0.0  # 이전 조향값 저장용 변수 추가

    def lane_changing_callback(self, msg):
        if self.lane_changing and not msg.data:
            # 차선 변경이 끝나 넘겨받는 순간: 변경 중(정면 복귀 때 선이 안 보임)의 검출 기록을 버림.
            # 안 버리면 최근 1초 검출률이 기준 아래라 넘겨받자마자 0.3초쯤 멈춤(10/17 18차)
            self.detect_hist = []
        self.lane_changing = bool(msg.data)

    def lane_change_callback(self, msg):
        if msg.data != self.lane_change_flag:
            self.get_logger().info(f'lane_change_flag -> {msg.data} ({"lane 1" if msg.data else "lane 2"})')
            self.steer_hist = []  # 기준선이 바뀌므로 이전 차선의 조향 이력은 버림
        self.lane_change_flag = msg.data

    def image_callback(self, msg):
        image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # lane_detect의 ROI/기준값은 640x480 좌표 기준. 640x480 영상은 DDS 전달 중 프레임이 많이 빠져서
        # 카메라를 320x240으로 받을 수 있게 하고, 여기서 640x480으로 늘려 기존 로직을 그대로 쓴다.
        if image.shape[1] != 640 or image.shape[0] != 480:
            image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_LINEAR)

        in_lane1 = self.lane_change_flag is True
        flip = in_lane1 and self.flip_on_lane_change
        if flip:
            image = cv2.flip(image, 1) # 이미지 반전
        if self.lane_changing:
            detector = self.detector_change
        else:
            detector = self.detector_l1 if in_lane1 else self.detector
        guide_x, guide_slope = (self.guide_x_l1, self.guide_slope_l1) if in_lane1 else (self.guide_x, self.guide_slope)
        if not in_lane1 and detector == 2:
            guide_x, guide_slope = self.guide_x_v2, self.guide_slope_v2
        right2 = self.right2_l1 if in_lane1 else self.right2
        roi_top = self.roi_top_l1 if in_lane1 else self.roi_top
        min_slope = self.min_slope
        if self.lane_changing:
            # 1차선으로 가는 중이면 flag가 True(in_lane1), 2차선으로 돌아가는 중이면 False
            roi_top_c = self.roi_top_change if in_lane1 else self.roi_top_change_l2
            min_slope_c = self.min_slope_change if in_lane1 else self.min_slope_change_l2
            if roi_top_c >= 0:
                roi_top = roi_top_c
            if min_slope_c >= 0.0:
                min_slope = min_slope_c

        result_img, mask_img, steer_rad = lane_detect(image, guide_x, guide_slope,
                                                       a_rad=self.a_rad_l1 if in_lane1 else self.a_rad,
                                                       b_rad=self.b_rad_l1 if in_lane1 else self.b_rad,
                                                       min_slope=min_slope, right1=self.right1, right2=right2,
                                                       roi_top=roi_top,
                                                       l_min=self.l_min, s_max=self.s_max, open_k=self.open_k,
                                                       max_blob=self.max_blob_l1 if in_lane1 else self.max_blob,
                                                       detector=detector)
        
        # --- 수정된 로직 ---
        if steer_rad is not None:
            # 새로운 값이 들어왔을 때만 업데이트
            # 한두 프레임 튀는 값(엉뚱한 선)을 걸러내는 중앙값 필터
            self.steer_hist.append(float(steer_rad))
            self.steer_hist = self.steer_hist[-self.steer_filter_n:]
            self.last_steer_rad = float(np.median(self.steer_hist))
            self.last_detect_t = time.time()
        else:
            # 조향은 마지막 값 유지하되, 차선 없는 상태로 계속 전진하면 위험하니 속도는 멈춤
            self.get_logger().warn('Lane lost - stopping throttle')
        
        # 현재(또는 유지된) 조향값 사용
        current_steer = self.last_steer_rad
        lost_hold_s = self.lost_hold_s_l1 if in_lane1 else self.lost_hold_s
        # lost_hold_s를 쓰는 경우, 그 시간 넘게 차선을 못 보면 마지막 조향(최대치일 수 있음)을 버리고 중앙으로
        if (lost_hold_s > 0.0 and steer_rad is None
                and (self.last_detect_t is None or time.time() - self.last_detect_t > lost_hold_s)):
            current_steer = 0.0
        elif in_lane1 and steer_rad is None:
            current_steer = float(np.clip(current_steer, -self.hold_steer_max_l1, self.hold_steer_max_l1))
        # -----------------        
        
        # --- [추가] 조향 부호 반전 로직 ---
        if flip:
            current_steer = -current_steer  # 이미지 반전에 맞춰 조향 방향도 반대로 뒤집음
        # -------------------------------
        
        steer_deg = np.degrees(current_steer)
        self.get_logger().info(f'Steering -> Rad: {current_steer:.3f}, Deg: {steer_deg:.1f}°')
        
        if self.debug_view:
            cv2.imshow("Lane Detection (Overlay)", result_img)
            cv2.imshow("White Mask (Binary)", mask_img)
            cv2.waitKey(1)
        
        # 제어 메시지 생성
        drive_msg = Twist()
        now = time.time()
        recently_seen = (self.last_detect_t is not None
                         and now - self.last_detect_t <= lost_hold_s)
        min_ratio, window_s = ((self.min_detect_ratio_l1, self.detect_window_s_l1) if in_lane1
                               else (self.min_detect_ratio, self.detect_window_s))
        self.detect_hist.append((now, steer_rad is not None))
        self.detect_hist = [(t, d) for t, d in self.detect_hist if now - t <= max(self.detect_window_s, self.detect_window_s_l1)]
        recent = [d for t, d in self.detect_hist if now - t <= window_s]
        detect_ratio = sum(recent) / len(recent)
        low_ratio = min_ratio > 0.0 and detect_ratio < min_ratio
        if low_ratio:
            self.get_logger().warn(f'Detect ratio {detect_ratio:.2f} < {min_ratio:.2f} - stopping throttle',
                                   throttle_duration_sec=0.5)
        drive_msg.linear.x = self.linear_speed if ((steer_rad is not None or recently_seen) and not low_ratio) else 0.0  # 차선 미검출 시 정지
        # 0.4 rad일 때 1000이 되도록 맵핑 (1000 / 0.4 = 2500)
        drive_msg.angular.z = -float(current_steer * 2500.0)
        
        self.cmd_pub.publish(drive_msg)
        self.detected_pub.publish(Bool(data=steer_rad is not None))
        self.slope_pub.publish(Float32(data=float(last_metrics['slope_avg']) if steer_rad is not None else -1.0))
        self.line_len_pub.publish(Float32(data=longest_line_len(image, self.l_min, self.s_max, self.open_k)))
        

def main(args=None):
    rclpy.init(args=args)
    node = LaneMaskingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
