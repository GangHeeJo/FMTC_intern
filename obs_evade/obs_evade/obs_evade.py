#!/usr/bin/env python3
# obs_evade: 라이다로 전방 장애물을 보고 반대 차선으로 옮겨가는 상태머신 노드.
# 평소엔 아무것도 안 하고(lane_trace가 /cmd_lane으로 주행), 장애물을 보면 /cmd_obs에 조향/속도를
# 직접 발행해서 decision_auto가 그걸 최우선으로 쓰게 함. 차선을 다 넘어가고 나면 다시 조용해짐.
#
# 상태 전환 흐름 (기본, mode='toggle'일 때):
#   LANE2 --(전방 장애물)--> TO_LANE1 --(새 차선 선 확인+정면 복귀)--> LANE1
#   LANE1 --(전방 장애물)--> TO_LANE2 --(새 차선 선 확인+정면 복귀)--> LANE2
# mode='return'이면 LANE1에서 "옆 장애물을 지나쳤는지"를 라이다로 봐서 자동으로 TO_LANE2로 돌아감.
# change_mode='camera'(기본)면 차선 변경 완료 판단을 "시간"이 아니라 lane_trace가 보내는
# /lane_detected(선 사라짐→새 선 잡힘), /lane_slope(기울기가 목표 차선 기준에 들어왔는지)로 함.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, UInt8, Float32MultiArray
import numpy as np
import time

# 차량 상태 (self.state가 이 5개 중 하나)
LANE2 = 'LANE2'              # 평소 주행 차선(오른쪽). lane_trace가 오른쪽 실선을 따라감
TO_LANE1 = 'TO_LANE1'        # 2차선 → 1차선 변경 중 (/cmd_obs 발행)
DECIDING = 'DECIDING'        # 출발 차선 자동 판단 중(start_lane=0). 장애물 감지 안 함
LANE1 = 'LANE1'              # 1차선 주행. lane_trace가 오른쪽 점선을 따라감 (lane_change_flag=True)
TO_LANE2 = 'TO_LANE2'        # 1차선 → 2차선 변경 중 (/cmd_obs 발행)


class ObstacleEvasionNode(Node):
    """node name: obs_evade. 라이다(/scan)와 lane_trace의 보조 토픽을 구독해서 상태(self.state)를
    관리하고, 10Hz 타이머(control_loop)가 그 상태에 맞는 /cmd_obs 명령을 발행함."""
    def __init__(self):
        super().__init__('obs_evade')

        # 1. 파라미터 설정
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_steer', 1000.0)
        # return: 장애물을 지나면 2차선으로 복귀 / toggle: 장애물을 만날 때마다 반대 차선으로
        self.declare_parameter('mode', 'return')
        # 차선 변경: 단계별 조향 비율(+1 = 왼쪽 최대)과 시간(초). 왼쪽 변경은 [+a, -a], 오른쪽 변경은 부호 반대로 씀
        self.declare_parameter('change_steer', [1.0, -1.0])
        self.declare_parameter('change_time_s', [2.5, 2.5])
        # timed 1→2 전용 시간. [-1.0]이면 change_time_s와 같게(10/19: 오른쪽으로 꺾을 때 덜 돌아 1→2가 덜 넘어감)
        self.declare_parameter('change_time_s_to_lane2', [-1.0])
        self.declare_parameter('change_speed', 0.7)  # max_speed에 곱하는 비율
        # camera: change_steer[0]으로 꺾다가 원래 보던 선이 사라진 뒤 새 차선의 선이 cam_found_frames 연속 잡히면
        #         변경 완료 → lane_trace에 넘김(방향 보정은 lane_trace가 함). timed: change_steer/change_time_s 단계대로
        self.declare_parameter('change_mode', 'camera')
        self.declare_parameter('cam_min_s', 1.0)        # 이 시간 전에는 완료로 보지 않음
        self.declare_parameter('cam_max_s', 5.0)        # 이 시간 안에 못 찾으면 경고 후 그냥 넘김
        # 목표 차선별 값(음수면 위 공통값). 2→1과 1→2를 따로 맞추기 위함(10/17)
        self.declare_parameter('cam_min_s_l1', -1.0)
        self.declare_parameter('cam_min_s_l2', -1.0)
        self.declare_parameter('cam_max_s_l1', -1.0)
        self.declare_parameter('cam_max_s_l2', -1.0)
        self.declare_parameter('cam_lost_frames', 5)    # 원래 선이 이만큼 연속 안 보여야 "사라짐"
        self.declare_parameter('cam_found_frames', 3)   # 새 선이 이만큼 연속 보여야 "잡힘"
        # 새 선을 잡은 뒤 반대로(change_steer[1]) 꺾어 정면 복귀: lane_trace가 보낸 선 기울기(/lane_slope)가
        # 그 차선의 기준 기울기(guide_slope)에 straighten_tol 안으로 들어오거나 지나치면 lane_trace에 넘김
        self.declare_parameter('straighten_slope_l1', 0.728)
        self.declare_parameter('straighten_slope_l2', 0.70)
        self.declare_parameter('straighten_tol', 0.1)
        # 차선별 허용 범위(음수면 straighten_tol). 2차선은 좁게: 선이 납작한 채 넘기면 lane_trace가 y=300 연장점을
        # 멀리 잡아 반대(오른쪽)로 꺾음(10/16 8차: 0.41에서 넘겨 경계선 쪽으로 감)
        self.declare_parameter('straighten_tol_l1', -1.0)
        self.declare_parameter('straighten_tol_l2', -1.0)
        self.declare_parameter('straighten_max_s', 3.0)  # 선이 안 보여 기울기를 모르면 이 시간 뒤 넘김
        # 차선별 최대 시간(음수면 straighten_max_s). 1차선은 정상 복귀가 4~6초인데 선이 안 보이면 8초를 다 꺾어
        # 첫 장애물 쪽으로 돌아감(10/17 7차) → 짧게
        self.declare_parameter('straighten_max_s_l1', -1.0)
        self.declare_parameter('straighten_max_s_l2', -1.0)
        # 새 선을 잡은 뒤 정면 복귀 전에 조향 0으로 직진하는 시간(초, 목표 차선별). 차가 크게 돌아 있으므로 직진하면
        # 새 차선 안쪽으로 더 들어감. 0이면 바로 정면 복귀(10/17: 커브 끝에서 바꾸면 1차선에 반만 들어가
        # 옆 장애물 차체를 선으로 잡음)
        self.declare_parameter('cam_cross_s_l1', 0.0)
        self.declare_parameter('cam_cross_s_l2', 0.0)
        # 새 선을 찾는 동안 꺾는 최대 시간(초, 목표 차선별). 넘으면 조향 0으로 직진하며 계속 찾음. 음수면 계속 꺾음.
        # 계속 꺾으면 출발 위치에 따라 너무 얕게(5차: 두 차선 걸침) 또는 너무 깊게(10차: 경계선을 정면으로 봐서
        # 인식 구역 위로 빠짐) 돌아감 → 일정 각도까지만 돌고 그 각도로 다가가게 함(10/17)
        self.declare_parameter('cam_turn_max_s_l1', -1.0)
        # 시작 차선(0, 1, 2). 1이면 1차선 주행 상태(lane_change_flag=True)로 시작 — 1→2 복귀만 반복 시험할 때(10/17)
        # 0이면 자동: lane_trace의 /lane_line_len(가장 긴 선 길이)을 start_frames장 모아 다수결로 판단.
        # start_len_px보다 길면 실선 = 2차선, 짧으면 점선 = 1차선(10/17 bag: 2차선 633~742, 1차선 94~346)
        self.declare_parameter('start_lane', 2)
        self.declare_parameter('start_len_px', 450.0)
        self.declare_parameter('start_frames', 15)
        self.declare_parameter('cam_turn_max_s_l2', -1.0)
        # cam_turn_max_s 뒤에 반대로 되꺾는 시간(초, 목표 차선별). 0이면 바로 직진. 2024 보고서처럼 좌→우로 방향을 되돌려
        # 새 차선 안쪽으로 파고들지 않게(10/19 회피 6차: 2초 꺾고 직진하자 1차선 실선 쪽으로 170px까지 들어감)
        self.declare_parameter('cam_back_s_l1', 0.0)
        self.declare_parameter('cam_back_s_l2', 0.0)
        # True면 1→2 변경을 2→1과 똑같은 절차로 하되 영상(lane_trace)과 조향 방향만 좌우 반전(원본 cam_lane_auto 방식).
        # 1→2 전용 값(*_l2 시간·기울기)은 안 쓰고 2→1 값(*_l1)을 그대로 씀. False면 10/17 방식(1→2를 따로 맞춘 값)(10/19)
        self.declare_parameter('mirror_to_lane2', True)
        # vision: 단계(change_steer 순서, 보통 [최대로 꺾기, 직진, 되꺾기])를 시간 대신 상단 카메라에 보이는 선 위치로 넘김.
        # 기준 = 사용자 시험 bag_obs_190819(2→1 2.3-2.0-2.5초, 1→2 2.7-2.0-2.0초)에서 각 단계가 끝날 때의 화면(10/19).
        # lane_trace /lane_lines = 반전 안 한 화면의 흰 선 [x(y=300), 기울기 dx/dy, 길이]. 길이 ≥ vis_len_min이고
        # 기울기가 vis_to{1,2}_k 범위인 선 중 가장 긴 것 = 새 차선 쪽 실선(2→1은 1차선 왼쪽 실선, 1→2는 2차선 오른쪽 실선).
        # 단계 i는 그 선의 x가 vis_to*_x[i]에 닿으면 끝(vis_to*_dir[i]: +1이면 x ≥, -1이면 x ≤).
        # 마지막 단계(되꺾기)는 기울기가 vis_to*_k_end보다 정면 쪽(|k| ≤ |k_end|)이 돼도 끝: 차가 옆으로 더 갔으면
        # 위치가 오기 전에 머리가 먼저 정면을 봄(bag_obs_190506·190124). 단계마다 vis_min_s 전에는 안 넘기고,
        # 그 방향의 시간 기준(change_time_s / change_time_s_to_lane2) + vis_extra_s가 지나면 시간 초과로 넘김
        self.declare_parameter('vis_len_min', 250.0)
        self.declare_parameter('vis_to1_k', [-2.0, -0.3])
        self.declare_parameter('vis_to1_x', [-95.0, 180.0, 95.0])
        self.declare_parameter('vis_to1_dir', [1, 1, -1])
        self.declare_parameter('vis_to1_k_end', -0.61)
        self.declare_parameter('vis_to2_k', [0.3, 2.0])
        self.declare_parameter('vis_to2_x', [604.0, 378.0, 495.0])
        self.declare_parameter('vis_to2_dir', [-1, -1, 1])
        self.declare_parameter('vis_to2_k_end', 0.66)
        self.declare_parameter('vis_min_s', [1.5, 1.0, 1.0])
        self.declare_parameter('vis_extra_s', 1.0)
        self.declare_parameter('vis_frames', 2)   # 조건이 이만큼 연속 맞아야 넘김
        # 전방 장애물 감지 구역(라이다 기준 정면은 -x): 거리 roi_near~roi_far(m), 좌우 ±roi_half_width(m)
        self.declare_parameter('roi_near', 1.0)
        self.declare_parameter('roi_far', 1.15)
        # 내 차선만 보도록 좁게(옆 차선의 장애물·주차 차량을 앞 장애물로 착각하지 않게). 원래 1.0
        self.declare_parameter('roi_half_width', 0.4)
        self.declare_parameter('min_points', 30)    # 구역 안 점 개수 기준
        self.declare_parameter('min_scans', 6)      # 연속 스캔 수 기준
        # return 모드: 차 오른쪽 옆 구역(앞 side_ahead ~ 뒤 side_behind, 오른쪽 side_lat_min~max m)에서
        # 장애물을 본 뒤 side_clear_scans번 연속 비면 복귀. 옆에서 한 번도 못 본 채 pass_timeout_s가 지나면 시간으로 복귀
        self.declare_parameter('side_ahead', 0.8)
        self.declare_parameter('side_behind', 0.2)
        self.declare_parameter('side_lat_min', 0.2)
        self.declare_parameter('side_lat_max', 1.0)
        self.declare_parameter('side_min_points', 10)
        self.declare_parameter('side_clear_scans', 5)
        self.declare_parameter('pass_timeout_s', 10.0)
        # False면 옆에서 봤는지와 상관없이 오른쪽 구역이 side_clear_scans번 연속 비면 바로 복귀(10초 대기 없음, 10/19 사용자).
        # 이때 side_ahead를 늘려 아직 오른쪽 앞에 있는 장애물도 구역 안에 들어오게 해야 함
        self.declare_parameter('side_require_seen', True)
        # 차선 변경 직후 이 시간 동안은 전방 감지 쉼(방금 피한 장애물 재감지 방지)
        self.declare_parameter('rearm_delay_s', 2.0)

        gp = lambda n: self.get_parameter(n).value  # 파라미터 값을 짧게 읽어오는 축약
        self.MAX_SPEED = float(gp('max_speed'))
        self.MAX_STEER = float(gp('max_steer'))
        self.MODE = str(gp('mode'))
        self.CHANGE_STEER = list(gp('change_steer'))
        self.CHANGE_TIME = list(gp('change_time_s'))
        t2 = list(gp('change_time_s_to_lane2'))
        self.CHANGE_TIME_BY = {TO_LANE1: self.CHANGE_TIME, TO_LANE2: self.CHANGE_TIME if t2 == [-1.0] else t2}
        self.CHANGE_SPEED = float(gp('change_speed'))
        self.CHANGE_MODE = str(gp('change_mode'))
        self.CAM_MIN_S, self.CAM_MAX_S = float(gp('cam_min_s')), float(gp('cam_max_s'))
        # by(name, base): "name_l1"/"name_l2" 파라미터를 읽어서 {LANE1: 값, LANE2: 값} 딕셔너리로.
        # 음수로 두면(기본값) 차선별로 안 나누고 공통값(base)을 그대로 씀
        by = lambda name, base: {lane: (float(gp(f'{name}_{suf}')) if float(gp(f'{name}_{suf}')) >= 0.0 else base)
                                 for lane, suf in ((LANE1, 'l1'), (LANE2, 'l2'))}
        self.CAM_MIN_BY, self.CAM_MAX_BY = by('cam_min_s', self.CAM_MIN_S), by('cam_max_s', self.CAM_MAX_S)
        self.CAM_LOST_FRAMES, self.CAM_FOUND_FRAMES = int(gp('cam_lost_frames')), int(gp('cam_found_frames'))
        self.STRAIGHTEN_SLOPE = {LANE1: float(gp('straighten_slope_l1')), LANE2: float(gp('straighten_slope_l2'))}
        self.STRAIGHTEN_TOL, self.STRAIGHTEN_MAX = float(gp('straighten_tol')), float(gp('straighten_max_s'))
        tol = lambda v: float(v) if float(v) >= 0.0 else self.STRAIGHTEN_TOL
        self.STRAIGHTEN_TOL_BY = {LANE1: tol(gp('straighten_tol_l1')), LANE2: tol(gp('straighten_tol_l2'))}
        smax = lambda v: float(v) if float(v) >= 0.0 else self.STRAIGHTEN_MAX
        self.STRAIGHTEN_MAX_BY = {LANE1: smax(gp('straighten_max_s_l1')), LANE2: smax(gp('straighten_max_s_l2'))}
        self.CAM_CROSS_S = {LANE1: float(gp('cam_cross_s_l1')), LANE2: float(gp('cam_cross_s_l2'))}
        self.CAM_TURN_MAX = {LANE1: float(gp('cam_turn_max_s_l1')), LANE2: float(gp('cam_turn_max_s_l2'))}
        self.CAM_BACK = {LANE1: float(gp('cam_back_s_l1')), LANE2: float(gp('cam_back_s_l2'))}
        self.ROI_NEAR, self.ROI_FAR = float(gp('roi_near')), float(gp('roi_far'))
        self.ROI_HALF_W = float(gp('roi_half_width'))
        self.MIN_POINTS, self.MIN_SCANS = int(gp('min_points')), int(gp('min_scans'))
        self.SIDE_AHEAD, self.SIDE_BEHIND = float(gp('side_ahead')), float(gp('side_behind'))
        self.SIDE_LAT_MIN, self.SIDE_LAT_MAX = float(gp('side_lat_min')), float(gp('side_lat_max'))
        self.SIDE_MIN_POINTS, self.SIDE_CLEAR_SCANS = int(gp('side_min_points')), int(gp('side_clear_scans'))
        self.PASS_TIMEOUT = float(gp('pass_timeout_s'))
        self.SIDE_REQUIRE_SEEN = bool(gp('side_require_seen'))
        self.REARM_DELAY = float(gp('rearm_delay_s'))
        self.MIRROR = bool(gp('mirror_to_lane2'))
        self.VIS = {n: gp(n) for n in ('vis_len_min', 'vis_min_s', 'vis_extra_s', 'vis_frames')}
        self.VIS_BY = {st: {n: gp(f'vis_{d}_{n}') for n in ('k', 'x', 'dir', 'k_end')}
                       for st, d in ((TO_LANE1, 'to1'), (TO_LANE2, 'to2'))}
        self.vis_lines = []       # [(x, k, len), ...] 마지막 /lane_lines
        self.vis_lines_t = 0.0
        self.vis_phase = 0        # vision 변경 단계(0, 1, 2)
        self.vis_phase_t = 0.0
        self.vis_hits = 0
        if self.MODE not in ('return', 'toggle'):
            raise ValueError("mode는 'return' 또는 'toggle'")
        if self.CHANGE_MODE not in ('camera', 'timed', 'vision'):
            raise ValueError("change_mode는 'camera', 'timed', 'vision'")
        if any(len(self.CHANGE_STEER) != len(t) for t in self.CHANGE_TIME_BY.values()):
            raise ValueError('change_steer와 change_time_s(_to_lane2)의 길이가 같아야 함')

        # 상태 제어 변수
        start_lane = int(gp('start_lane'))
        if start_lane not in (0, 1, 2):
            raise ValueError('start_lane은 0(자동), 1, 2')
        self.START_LEN_PX, self.START_FRAMES = float(gp('start_len_px')), int(gp('start_frames'))
        self.start_votes = []
        self.state = {0: DECIDING, 1: LANE1, 2: LANE2}[start_lane]
        self.state_start = time.time()
        self.change_step = -1
        self.obstacle_count = 0
        self.side_seen = False       # return 모드: 1차선에서 옆 장애물을 봤는지
        self.side_clear_count = 0
        self.cam_lost_seen = False   # camera 변경: 원래 선이 사라졌는지
        self.cam_lost_count = 0
        self.cam_found_count = 0
        self.blocked = False         # 1차선 앞과 2차선 옆이 모두 막혀 정지 중
        self.straighten_start = None  # camera 변경: 정면 복귀 시작 시각(None이면 아직 꺾는 중)
        self.cross_start = None       # camera 변경: 새 선을 잡은 뒤 직진 시작 시각
        self.straighten_err0 = None   # 정면 복귀 시작 때의 기울기 오차 부호
        self.last_slope, self.last_slope_t = -1.0, 0.0

        # 2. Publisher & Subscriber 설정
        # 평상시 주행은 다른 노드에서 하므로, 장애물 전용 토픽 /cmd_obs 사용
        self.cmd_obs_pub = self.create_publisher(Twist, '/cmd_obs', 10)
        # lane_change_flag: True = 1차선 주행 중 (lane_trace가 1차선 기준값 사용)
        self.flag_pub = self.create_publisher(Bool, 'lane_change_flag', 10)
        # 차선 변경 중이면 True: lane_trace가 이 동안 인식 조건을 완화해 새 차선의 선을 일찍 잡음
        self.changing_pub = self.create_publisher(Bool, '/lane_changing', 10)
        # 1→2 변경 중(mirror_to_lane2)이면 True: lane_trace가 영상을 좌우 반전하고 2→1 변경 때의 인식 조건을 씀
        self.mirror_pub = self.create_publisher(Bool, '/lane_mirror', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.detected_sub = self.create_subscription(Bool, '/lane_detected', self.detected_callback, 10)
        self.slope_sub = self.create_subscription(Float32, '/lane_slope', self.slope_callback, 10)
        self.line_len_sub = self.create_subscription(Float32, '/lane_line_len', self.line_len_callback, 10)
        self.lines_sub = self.create_subscription(Float32MultiArray, '/lane_lines', self.lines_callback, 10)
        # 횡단보도 구간(decision_auto /crosswalk_active)에서는 앞 장애물 감지를 쉼: 정지선에서 횡단보도 너머 구조물을
        # 장애물로 잡아 빨간불에 차선 변경을 시작함(10/19 bag_full_195127)
        self.cw_active_t = 0.0
        self.cw_sub = self.create_subscription(Bool, '/crosswalk_active', self.cw_callback, 10)
        # 자동(AUTO=1) 모드에서만 장애물 감지. 수동으로 차를 옮기다 장애물에 다가가도 회피 상태로 넘어가지 않게(10/19:
        # 시험 사이에 obs_evade가 LANE1로 넘어가 있어 lane_trace가 2차선에서 반전 영상으로 1차선 실선을 찾음)
        self.drive_mode = None
        self.mode_sub = self.create_subscription(UInt8, '/drive_mode', self.mode_callback, 10)

        # 제어 루프 (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Obstacle Evasion Node Started: mode={self.MODE} change_mode={self.CHANGE_MODE} '
            f'(cam to L1 {self.CAM_MIN_BY[LANE1]}~{self.CAM_MAX_BY[LANE1]}s, '
            + (f'to L2 = mirrored L1) ' if self.MIRROR else f'to L2 {self.CAM_MIN_BY[LANE2]}~{self.CAM_MAX_BY[LANE2]}s) ')
            + f'change steer={self.CHANGE_STEER} '
            f'time={self.CHANGE_TIME}s to L2 {self.CHANGE_TIME_BY[TO_LANE2]}s speed={self.CHANGE_SPEED} roi={self.ROI_NEAR}~{self.ROI_FAR}m')

    def set_state(self, state):
        """상태를 바꾸고 그 상태에서 쓰는 타이머/카운터를 전부 초기화함. 상태가 바뀔 때마다 꼭 이걸 통해서
        바꿔야 함(직접 self.state = ... 로 바꾸면 이전 상태의 카운터가 남아서 오작동할 수 있음)"""
        self.state = state
        self.state_start = time.time()
        self.change_step = -1
        self.obstacle_count = 0
        self.side_seen = False
        self.side_clear_count = 0
        self.cam_lost_seen = False
        self.cam_lost_count = 0
        self.cam_found_count = 0
        self.straighten_start = None
        self.cross_start = None
        self.straighten_err0 = None
        self.vis_phase = 0
        self.vis_phase_t = time.time()
        self.vis_hits = 0
        self.get_logger().warn(f'State -> {state}')

    def lines_callback(self, msg):
        d = list(msg.data)
        self.vis_lines = [(d[i], d[i + 1], d[i + 2]) for i in range(0, len(d) - 2, 3)]
        self.vis_lines_t = time.time()

    def vis_condition(self, phase):
        """지금 화면에서 vision 단계 phase의 끝 조건이 맞는지, 그리고 본 값(x, k)"""
        V = self.VIS_BY[self.state]
        k0, k1 = V['k']
        cand = [(l, x, k) for x, k, l in self.vis_lines if k0 <= k <= k1 and l >= self.VIS['vis_len_min']]
        if not cand:
            return False, None
        _, x, k = max(cand)
        ok = (x - V['x'][phase]) * V['dir'][phase] >= 0
        if phase == len(V['x']) - 1:
            ok = ok or abs(k) <= abs(V['k_end'])
        return ok, (x, k)

    def cw_callback(self, msg):
        if msg.data:
            self.cw_active_t = time.time()

    def mode_callback(self, msg):
        self.drive_mode = int(msg.data)

    def slope_callback(self, msg):
        """lane_trace가 보낸 최근 검출 기울기(/lane_slope) 저장. 정면 복귀 완료 판단(control_loop)에 씀"""
        self.last_slope, self.last_slope_t = float(msg.data), time.time()

    def detected_callback(self, msg):
        """camera 변경 중: lane_trace의 선 검출 여부로 '원래 선 사라짐 → 새 선 잡힘'을 셈"""
        if self.CHANGE_MODE != 'camera' or self.state not in (TO_LANE1, TO_LANE2):
            return
        if not self.cam_lost_seen:
            self.cam_lost_count = 0 if msg.data else self.cam_lost_count + 1
            if self.cam_lost_count >= self.CAM_LOST_FRAMES:
                self.cam_lost_seen = True
                self.get_logger().info('Lane change: previous line lost')
        else:
            self.cam_found_count = self.cam_found_count + 1 if msg.data else 0

    def line_len_callback(self, msg):
        """출발 차선 자동 판단: 선이 보이는 프레임만 모아 다수결"""
        if self.state != DECIDING or msg.data <= 0.0:
            return
        self.start_votes.append(msg.data > self.START_LEN_PX)
        if len(self.start_votes) >= self.START_FRAMES:
            solid = sum(self.start_votes)
            lane = LANE2 if solid * 2 > len(self.start_votes) else LANE1
            self.get_logger().warn(f'Start lane auto: {lane} (solid {solid}/{len(self.start_votes)} frames, '
                                   f'>{self.START_LEN_PX:.0f}px = solid line)')
            self.set_state(lane)

    def scan_callback(self, msg):
        """라이다 데이터로 전방 장애물 / (return 모드) 옆 장애물 통과 판단"""
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        valid_idx = (ranges > 0.1) & (ranges < 5.0)
        # 라이다는 차 기준 180° 돌아 달려 있음: 차 앞 = 라이다 -x, 차 오른쪽 = 라이다 +y (10/16 시험으로 확인)
        x = ranges[valid_idx] * np.cos(angles[valid_idx])
        y = ranges[valid_idx] * np.sin(angles[valid_idx])
        ahead, right = -x, y

        if self.state == LANE1 and self.MODE == 'return':
            side_idx = ((ahead < self.SIDE_AHEAD) & (ahead > -self.SIDE_BEHIND)
                        & (right > self.SIDE_LAT_MIN) & (right < self.SIDE_LAT_MAX))
            if np.sum(side_idx) >= self.SIDE_MIN_POINTS:
                if not self.side_seen:
                    self.get_logger().info('Obstacle beside the car')
                self.side_seen = True
                self.side_clear_count = 0
            elif self.side_seen or not self.SIDE_REQUIRE_SEEN:
                self.side_clear_count += 1
        side_occupied = ((self.side_seen or not self.SIDE_REQUIRE_SEEN) and self.side_clear_count < self.SIDE_CLEAR_SCANS)

        # 전방 장애물: 2차선이든 1차선이든 감지되면 반대 차선으로 (두 모드 공통)
        # 차선 변경 직후 rearm_delay_s 동안은 쉼(방금 피한 장애물이 옆 차선 앞쪽에 보여 재감지되는 것 방지)
        self.blocked = False
        # 모드를 모르면(cmd_mux는 바뀔 때만 알림) 감지 안 함: obs_run을 켠 뒤 A를 누르면 그때 알게 됨(10/19: 모드 모른 채
        # 차를 옮기는 동안 회피로 넘어감)
        if self.drive_mode != 1 or time.time() - self.cw_active_t < 0.5:
            self.obstacle_count = 0
        elif self.state in (LANE2, LANE1) and time.time() - self.state_start >= self.REARM_DELAY:
            roi_idx = (ahead > self.ROI_NEAR) & (ahead < self.ROI_FAR) & (np.abs(right) < self.ROI_HALF_W)
            if np.sum(roi_idx) >= self.MIN_POINTS:
                self.obstacle_count += 1
            else:
                self.obstacle_count = 0
            if self.obstacle_count >= self.MIN_SCANS:
                if self.state == LANE1 and side_occupied:
                    # 앞(1차선)과 오른쪽 옆(2차선)이 모두 막힘 → 옆이 빌 때까지 정지
                    self.blocked = True
                    self.get_logger().error('Both lanes blocked -> holding', throttle_duration_sec=1.0)
                    return
                self.get_logger().warn('Obstacle Detected!')
                self.set_state(TO_LANE1 if self.state == LANE2 else TO_LANE2)
                return

        if self.state == LANE1 and self.MODE == 'return':
            if (self.side_seen or not self.SIDE_REQUIRE_SEEN) and self.side_clear_count >= self.SIDE_CLEAR_SCANS:
                self.get_logger().warn(f'Right side clear (obstacle seen beside: {self.side_seen}) -> back to lane 2')
                self.set_state(TO_LANE2)
            elif not self.side_seen and time.time() - self.state_start > self.PASS_TIMEOUT:
                # 옆에서 한 번이라도 봤으면 비워질 때까지 기다림(아직 옆에 있는데 복귀하면 부딪침)
                self.get_logger().warn(f'Obstacle not seen beside within {self.PASS_TIMEOUT}s -> back to lane 2')
                self.set_state(TO_LANE2)

    def control_loop(self):
        """10Hz 타이머. 지금 상태(self.state)에 따라 해야 할 일을 전부 여기서 결정함:
        차선 변경 중이 아니면 아무것도 안 하고, 변경 중이면 camera/timed 방식에 맞게
        /cmd_obs(조향+속도)를 계산해서 보냄. 상태 전환 자체는 scan_callback(장애물 보고 시작)과
        이 함수 안(새 선 찾기 완료, 정면 복귀 완료 등)에서 일어남."""
        # 1. 플래그 발행: 1차선이거나 1차선으로 가는 중이면 True
        flag_msg = Bool()
        flag_msg.data = self.state in (TO_LANE1, LANE1)
        self.changing_pub.publish(Bool(data=self.state in (TO_LANE1, TO_LANE2)))
        self.mirror_pub.publish(Bool(data=self.MIRROR and self.state == TO_LANE2))
        self.flag_pub.publish(flag_msg)

        # 양쪽 차선이 다 막혔으면 정지 명령(/cmd_obs는 decision_auto에서 차선 추종보다 우선)
        if self.blocked:
            self.cmd_obs_pub.publish(Twist())
            return

        # 2. 차선 변경 중이면 경과 시간으로 현재 단계를 찾아 /cmd_obs 발행
        if self.state not in (TO_LANE1, TO_LANE2):
            return  # 평상시에는 /cmd_obs에 아무것도 publish하지 않음 → decision_auto가 /cmd_lane 사용

        sign = 1.0 if self.state == TO_LANE1 else -1.0  # 오른쪽 변경은 조향 부호 반대
        elapsed = time.time() - self.state_start
        next_state = LANE1 if self.state == TO_LANE1 else LANE2
        # 시간·기울기 값을 고를 차선. 반전 방식이면 1→2도 2→1(LANE1) 값을 그대로 씀 (조향 부호만 sign으로 반대)
        pk = LANE1 if self.MIRROR else next_state

        if self.CHANGE_MODE == 'vision':
            now = time.time()
            ph = self.vis_phase
            pe = now - self.vis_phase_t
            fresh = now - self.vis_lines_t < 0.3
            ok, val = self.vis_condition(ph) if fresh else (False, None)
            self.vis_hits = self.vis_hits + 1 if (ok and pe >= self.VIS['vis_min_s'][ph]) else 0
            timeout = pe >= self.CHANGE_TIME_BY[self.state][ph] + self.VIS['vis_extra_s']
            if self.vis_hits >= int(self.VIS['vis_frames']) or timeout:
                why = f'시간 초과 {pe:.1f}s' if (timeout and self.vis_hits < int(self.VIS['vis_frames'])) else f'화면 조건 맞음 (x {val[0]:.0f}, k {val[1]:.2f})'
                self.get_logger().warn(f'{self.state} 단계 {ph + 1}/{len(self.CHANGE_STEER)} 끝: {why}, {pe:.1f}s')
                self.vis_phase += 1
                self.vis_phase_t = now
                self.vis_hits = 0
                if self.vis_phase >= len(self.CHANGE_STEER):
                    self.set_state(next_state)
                    return
                ph = self.vis_phase
            twist_msg = Twist()
            twist_msg.linear.x = self.MAX_SPEED * self.CHANGE_SPEED
            twist_msg.angular.z = self.MAX_STEER * sign * self.CHANGE_STEER[ph]
            self.cmd_obs_pub.publish(twist_msg)
            return

        if self.CHANGE_MODE == 'camera':
            if self.straighten_start is not None:
                # 정면 복귀: 반대로 꺾다가 선 기울기가 기준에 들어오거나 지나치면 lane_trace에 넘김
                st_elapsed = time.time() - self.straighten_start
                target = self.STRAIGHTEN_SLOPE[pk]
                fresh = time.time() - self.last_slope_t < 0.3 and self.last_slope >= 0.0
                if fresh:
                    err = self.last_slope - target
                    if self.straighten_err0 is None:
                        self.straighten_err0 = np.sign(err)
                    if abs(err) < self.STRAIGHTEN_TOL_BY[pk] or np.sign(err) != self.straighten_err0:
                        self.get_logger().warn(f'Straightened in {st_elapsed:.1f}s (slope {self.last_slope:.2f}, '
                                               f'target {target:.2f}) -> lane_trace')
                        self.set_state(next_state)
                        return
                if st_elapsed > self.STRAIGHTEN_MAX_BY[pk]:
                    self.get_logger().error(f'Not straightened within {self.STRAIGHTEN_MAX_BY[pk]}s '
                                            f'-> handing over anyway')
                    self.set_state(next_state)
                    return
                back = self.CHANGE_STEER[1] if len(self.CHANGE_STEER) > 1 else -self.CHANGE_STEER[0]
                twist_msg = Twist()
                twist_msg.linear.x = self.MAX_SPEED * self.CHANGE_SPEED
                twist_msg.angular.z = self.MAX_STEER * sign * back
                self.cmd_obs_pub.publish(twist_msg)
                return
            if self.cross_start is not None:
                # 새 선을 잡은 뒤 직진: 새 차선 안쪽으로 더 들어간 다음 정면 복귀
                if time.time() - self.cross_start >= self.CAM_CROSS_S[pk]:
                    self.get_logger().warn(f'Crossed for {self.CAM_CROSS_S[pk]:.1f}s -> straightening')
                    self.straighten_start = time.time()
                    return
                twist_msg = Twist()
                twist_msg.linear.x = self.MAX_SPEED * self.CHANGE_SPEED
                twist_msg.angular.z = 0.0
                self.cmd_obs_pub.publish(twist_msg)
                return
            if self.cam_found_count >= self.CAM_FOUND_FRAMES and elapsed >= self.CAM_MIN_BY[pk]:
                if self.CAM_CROSS_S[pk] > 0.0:
                    self.get_logger().warn(f'New lane line found in {elapsed:.1f}s -> crossing straight')
                    self.cross_start = time.time()
                else:
                    self.get_logger().warn(f'New lane line found in {elapsed:.1f}s -> straightening')
                    self.straighten_start = time.time()
                return
            if elapsed > self.CAM_MAX_BY[pk]:
                self.get_logger().error(f'New lane line not found within {self.CAM_MAX_BY[pk]}s '
                                        f'(previous line lost: {self.cam_lost_seen}) -> handing over anyway')
                self.set_state(next_state)
                return
            twist_msg = Twist()
            twist_msg.linear.x = self.MAX_SPEED * self.CHANGE_SPEED
            turn_max = self.CAM_TURN_MAX[pk]
            back = self.CHANGE_STEER[1] if len(self.CHANGE_STEER) > 1 else -self.CHANGE_STEER[0]
            if 0.0 <= turn_max <= elapsed < turn_max + self.CAM_BACK[pk]:
                twist_msg.angular.z = self.MAX_STEER * sign * back  # 반대로 되꺾어 방향을 되돌림
            elif 0.0 <= turn_max <= elapsed:
                twist_msg.angular.z = 0.0  # 정해진 각도까지 돌았으면 직진하며 새 선을 찾음
            else:
                twist_msg.angular.z = self.MAX_STEER * sign * self.CHANGE_STEER[0]
            self.cmd_obs_pub.publish(twist_msg)
            return

        times = self.CHANGE_TIME_BY[self.state]
        step, t_end = None, 0.0
        for i, dur in enumerate(times):
            t_end += dur
            if elapsed < t_end:
                step = i
                break

        if step is None:
            self.set_state(next_state)
            return
        if step != self.change_step:
            self.change_step = step
            self.get_logger().info(f'{self.state} step {step + 1}/{len(times)}: '
                                   f'steer {sign * self.CHANGE_STEER[step]:+.1f} for {times[step]}s')
        twist_msg = Twist()
        # 조향 +1이 왼쪽 최대(+max_steer)
        twist_msg.linear.x = self.MAX_SPEED * self.CHANGE_SPEED
        twist_msg.angular.z = self.MAX_STEER * sign * self.CHANGE_STEER[step]
        self.cmd_obs_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleEvasionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
