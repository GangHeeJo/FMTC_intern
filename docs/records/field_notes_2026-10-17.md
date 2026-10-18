# Field Notes — 2026-10-17 (장애물 차선 변경 완성, 차선 검출 개선)

> 한 장 요약은 [handoff_2026-10-17](../summary/handoff_2026-10-17.md). 전날 기록은 [field_notes_2026-10-16](field_notes_2026-10-16.md) (특히 7장).

반영 상태 표기: **커밋** = 이 문서와 함께 `gyungyoon`에 커밋 / **시스템** = 이 노트북 OS 설정(재부팅 시 원래대로) / **임시** = 실행 중에만 유효 / **미해결**

작업: 도경윤(실차) + Claude(분석·코드). 시간: 10/17 19:10 ~ 22:45. 차: 노트북 직결, 라이다·카메라 2대·Arduino·Xbox 패드.

---

## 0. 결론

- **장애물 2개 차선 변경 전체 순서 성공.** 2차선 출발(2차선 → 1차선 → 2차선 → 2차선 계속)과 1차선 출발(1차선 → 2차선 → 2차선 계속) 모두. 설정은 `s1_stack/config/obstacle_1017.yaml`에 저장.
- `lane_trace`에 **모양 기반 선 검출(방식 2)** 추가. 흰 덩어리의 굵기·길이·모양으로 점선/실선/주차선/흰 차를 구분. 1차선 점선 검출 40~60% → 98~100%, 상자 옆 2차선 구간에서 바깥 선을 잡던 문제 해결.
- **신호등 감지 실차 검증 완료.** 빨간불에서 정지하고 신호가 바뀌면 다시 출발함(light 시험, 도경윤 확인). 횡단보도 정지도 전체 시험 2·3차에서 확인.
- 남은 큰 문제: **밤 조명에서 흰 선 색이 바뀌어(푸르스름/신호등 근처 붉음) 검출이 떨어짐**, `sign_detect`가 1~3초씩 멈춤, 출발 차선 자동 판단이 일부 구간에서 틀림. → 6장.

---

## 1. 하드웨어·환경 사건

| 시각 | 사건 | 원인 → 조치 | 반영 |
|---|---|---|---|
| 19:15 | 조이스틱 연결은 되는데 입력 없음, 불 깜빡임 | 패드가 USB 연결을 마치지 못함. `xpad` 다시 붙여도 안 됨 → **패드 끄고 켜기 + USB 다시 꽂기**로 해결. 확인은 `/dev/input/js0` 직접 읽기 | 임시 |
| 19:50 | 주행 중 서다 가다(신호 토픽 끊김) | 우분투 자동 업데이트(`unattended-upgrade`, snapd)가 CPU 사용 + 전방 카메라 `exposure_dynamic_framerate=1`(어두우면 프레임 감소) → `systemctl stop apt-daily(-upgrade).service/.timer`, `v4l2-ctl -d /dev/cam_front -c exposure_dynamic_framerate=0` | 시스템(재부팅 시 원래대로) |
| 21:10 | 배터리 교체 후 바퀴(조향)가 명령 0인데 제멋대로 움직임 | Arduino 조향은 가변저항(A4) 피드백으로 가운데를 맞추는 구조. **A4·8번 선 접촉 불량**(10/16에도 다시 꽂음) → 다시 꽂음. **고정 필요** | 미해결(배선) |
| 21:35~ | 1차선 주행 중 0.5~3.4초씩 정지 | `sign_detect`(YOLO, CPU)가 가끔 멈춰 `decision_auto`의 1초 신호 시간 초과(STALE) → 장애물 시험 동안 `decision_auto`를 `require_signals:=false`로 따로 띄움. `lane_trace`·`obs_evade`·녹화는 `taskset -c 0-5`, `sign_detect`는 6~11 | 임시 |
| 21:56 | 정지 버튼 혼동 | `cmd_mux` 주석은 "X"였지만 **정지는 Y(3번)**, X는 2번 → 주석 수정 | 커밋 |
| 22:00 | 디스크 가득(녹화 중단) | 49GB 중 bag가 대부분. 녹화 전 `~/fmtc_session_1017/free_space.sh`(5GB 미만이면 오래된 bag부터 삭제)로 운영. 10/16 bag 전부와 10/17 초반 bag 삭제. 전방 카메라는 압축 토픽으로 녹화 | 로컬 스크립트 |

---

## 2. 코드 변경 (모두 새 파라미터, 기본값 = 기존 동작)

### 2-1. `lane_trace/lane_trace/lane_trace.py`

| 추가 | 내용 |
|---|---|
| `lane_detect_v2` (`detector`=2) | 보닛을 고정 다각형(`HOOD_POLY`, bag 650장에서 70% 이상 흰색인 영역)으로 지우고, 흰 덩어리마다 굵기(거리변환)·길이(직선 맞춤)·채움비·볼록도를 재서 거름. 주차선 = 가늘다(`open_k`), 흰 차 = 굵다(>40px) 또는 들쭉날쭉(fill <0.75, solid <0.85), 점선 = 굵기 20~33·꽉 찬 직사각형, 실선 = 길다(>300px, 모양 검사 생략). `guide_x`에 가장 가까운 선을 고르고 **한 줄 위의 토막을 모아 다시 맞춤**(점선 사이 빈 구간에도 끊기지 않음). 기존의 빨간 대각선 제외 없음 |
| `detector`, `detector_l1`, `detector_change` | 2차선 주행 / 1차선 주행 / 차선 변경 중에 쓸 방식. 차선 변경 중은 1 유지(`obs_evade` 시간 값이 방식 1 기준) |
| `guide_x_v2`, `guide_slope_v2` | 방식 2로 2차선을 따라갈 때 기준값(384/0.73). 방식 2는 휜 실선 전체를 맞춰 방식 1보다 약 20px 왼쪽·0.03 가파르게 읽음 → 이 값이면 2차선 bag 5구간에서 조향 중앙값 차이 ±70/1000 |
| `max_blob`, `max_blob_l1` | (방식 1용) 이 면적보다 큰 흰 덩어리 제거. 1차선 직후 옆 장애물 차체 오인 방지 |
| `roi_top_change_l2`, `min_slope_change_l2` | 1→2 복귀 때만 쓰는 새 선 인식 조건(기본 = 2→1과 같음) |
| `/lane_line_len` 토픽, `longest_line_len()` | 화면에서 가장 긴 선 길이. 실선(2차선) 633~742px, 점선(1차선) 94~346px → 출발 차선 자동 판단에 사용 |
| `lane_changing_callback` | 차선 변경이 끝나 넘겨받는 순간 검출 기록(`detect_hist`)을 비움. 안 비우면 넘겨받자마자 0.3초 정지(18차) |

### 2-2. `obs_evade/obs_evade/obs_evade.py` (10/16 변경 포함)

| 추가 | 내용 |
|---|---|
| `cam_cross_s_l1/_l2` | 새 차선 선을 찾은 뒤 정면 복귀 전에 조향 0으로 직진하는 시간. 1차선에 반만 들어가 옆 장애물 차체를 선으로 잡던 문제(2~4차) 해결 |
| `cam_turn_max_s_l1/_l2` | 새 선을 찾는 동안 꺾는 최대 시간. 넘으면 직진하며 찾음. 계속 꺾으면 출발 위치에 따라 너무 얕게(두 차선 걸침) 또는 너무 깊게(경계선을 정면으로 봐서 인식 구역 위로 빠짐) 돌아감 |
| `cam_min_s_l1/_l2`, `cam_max_s_l1/_l2` | 새 선 인정 최소/최대 시간을 방향별로. 1→2는 우회전이 끝나기 전 보이는 선 = 이전 점선(6·15차 오인) → 6.0 |
| `straighten_max_s_l1/_l2` | 정면 복귀 최대 시간 방향별. 1차선 쪽 5초(선이 안 보이면 8초 동안 첫 장애물 쪽으로 돌아감, 7차) |
| `start_lane` (0/1/2), `start_len_px`, `start_frames` | 시작 차선. 0이면 `/lane_line_len` 15프레임 다수결(>450px = 실선 = 2차선) |

### 2-3. 기타

- `s1_stack/s1_stack/cmd_mux.py`: 정지 버튼 주석 X → Y.
- `s1_stack/launch/s1_stack.launch.py`: 차선 카메라 320×240 (10/16).
- `scripts/lane_tuning/bag_montage.py`: 320×240 bag을 640×480으로 늘려서 처리(빠져 있어 결과가 틀렸음).
- `scripts/lane_tuning/bag_param_sweep.py`, `calib_pose.py`: 640×480 리사이즈, `l_min`/`s_max` (10/16).
- **`s1_stack/config/obstacle_1017.yaml`**: 오늘 확정 설정(3장).

---

## 3. 확정 설정과 실행 명령

`s1_stack/config/obstacle_1017.yaml` (값마다 이유 주석 있음). 핵심:

| 구분 | 값 |
|---|---|
| 2차선 주행 | `detector` 2, `guide_x_v2` 384, `guide_slope_v2` 0.73, `b_rad` 0.005, `min_detect_ratio` 0.3, `detect_window_s` 1.0 |
| 1차선 주행 | `detector_l1` 2, `guide_x_l1` 400, `guide_slope_l1` 0.728, `b_rad_l1` 0.0055, `lost_hold_s_l1` 4.0 |
| 흰 선 | `l_min` 130, **`s_max` 30**(밤 조명에서 점선 채도 22 → 20이면 통째로 걸러짐), `open_k` 13(9면 주차 칸 세로선을 점선으로 착각) |
| 2→1 (`_l1`) | `cam_min_s` 3.0, `cam_max_s` 8.0, `cam_cross_s` 1.5, `straighten_tol` 0.3, `straighten_max_s` 5.0 |
| 1→2 (`_l2`) | `cam_min_s` 6.0, `cam_max_s` 10.0, `cam_turn_max_s` 6.0, `cam_cross_s` 4.0, `straighten_tol` 0.12, `straighten_max_s` 8.0 |
| 감지 구역 | `roi_near` 1.3, `roi_far` 1.45, 좌우 ±0.4 (오른쪽만 좁히는 안은 도경윤 결정으로 적용 안 함) |
| `decision_auto` | `require_signals: false` (장애물 시험 동안만. 신호 **값**(횡단보도·빨간불)은 그대로 적용됨, 끊김만 무시) |

```bash
# 터미널 1
ros2 launch s1_stack s1_stack.launch.py
v4l2-ctl -d /dev/cam_lane -c brightness=128 -c exposure_dynamic_framerate=0
v4l2-ctl -d /dev/cam_front -c exposure_dynamic_framerate=0
# (장애물 시험) launch 안의 decision_auto를 끄고 신호 끊김 무시로 다시 띄움
kill -INT $(pgrep -f "lib/s1_stack/decision_aut[o]"); taskset -c 0-5 ros2 run s1_stack decision_auto --ros-args -p require_signals:=false
# 터미널 2
OMP_NUM_THREADS=6 taskset -c 6-11 ros2 run sign_detect sign_detect --ros-args -p model_path:=$HOME/FMTC_intern/sign_detect/best_0808.pt
# 터미널 3
taskset -c 0-5 ros2 run lane_trace lane_trace --ros-args --params-file ~/FMTC_intern/s1_stack/config/obstacle_1017.yaml
# 터미널 4 — 차를 출발 위치에 놓은 뒤(장애물이 감지 구역 밖), 시험마다 새로
taskset -c 0-5 ros2 run obs_evade obs_evade --ros-args --params-file ~/FMTC_intern/s1_stack/config/obstacle_1017.yaml -p start_lane:=2   # 1차선 출발이면 1
```

- 시험 후 `obs_evade`를 끄고 `ros2 topic pub --once /lane_change_flag std_msgs/msg/Bool "{data: false}"`.
- 조이스틱: **A 자동, B 수동, Y 정지**. 노트북에서 정지: `ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0,0,1,0,0,1,0,0], buttons: [0,0,0,1,0,0,0,0,0,0,0,0]}"` (버튼 12개여야 먹음).
- 녹화 전 `~/fmtc_session_1017/free_space.sh`.

---

## 4. 시험 기록 (bag은 `~/fmtc_session_1017/`, 일부 삭제)

`cmd_out`의 0.7은 명령값이고 실제 속도는 약 0.2m/s.

| 시험 | 바꾼 것 | 결과 → 조치 |
|---|---|---|
| curve1 | 커브 시작 출발, `b_rad` 0.004 | 17초 후 커브 바깥으로 밀려 선 놓침(선이 빨간 대각선 제외 구역으로) → 0.005 |
| curve2/3 | `b_rad` 0.005, 안전장치 0.5/0.5s → 0.3/1.0s | 커브 완주, 잠깐 정지 2번 → 0번 |
| obs1 | 장애물 시험 | 장애물에 도달 전 커브에서 선 놓침(위와 같음) |
| obs2 | | 커브 끝에서 감지 → 1차선 반만 진입, 옆 흰 차 차체를 점선으로 추종 → `cam_cross_s_l1` 추가 |
| obs3 | | `unattended-upgrade` CPU 사용으로 서다 가다 → 업데이트 정지 |
| obs4 | 직진 0.7s | 여전히 반만 진입, 점선이 인식 구역 밖 |
| obs5 | 직진 1.5s, `max_blob_l1` | **1차선 진입·주행·두 번째 감지 성공**. 복귀는 누운 경계선(0.13)을 일찍 잡아 정면 복귀 미완 |
| obs6 | 복귀 `min_slope_change_l2` 0.2, `cam_cross_s_l2` 0.7 | 복귀 1초 만에 이전 점선을 새 선으로 오인 → `cam_min_s` 3.0 |
| obs7 | | 정면 복귀 중 점선 안 보임, 8초 꺾어 첫 장애물 쪽으로 → `straighten_max_s_l1` 5.0. 사후 분석: 주차 칸 세로선 오인 |
| obs8 | | 새 선 = **주차 칸 세로선**(도경윤 관찰) → `open_k` 13 |
| obs9 | `open_k` 13 | 1차선 변경 정상. 1차선 주행 중 점선 빈 구간 4초 → 정지 → **방식 2 개발**(오프라인 검출 40% → 100%) |
| obs10 | `detector_l1` 2 | **1차선 주행 완벽**. 복귀 때 경계선 못 찾음(`min_slope_change_l2` 0.2가 원인) |
| obs11~14 | 복귀 "6초 꺾기 → 직진하며 찾기", 직진 0.7→1.2→3.0s | 13차부터 복귀 완료, **14차 첫 전체 성공**(복귀 후 2차선 25초) |
| obs15 | 1차선 출발, 직진 4.5s | 우회전 3초 만에 이전 점선 오인 → 장애물로 직진(0.28m에서 정지) → `cam_min_s_l2` 6.0 |
| obs16 | | 복귀 정상. `sign_detect` 끊김으로 서다 가다 → `require_signals:=false` |
| obs17/18 | 직진 3.5 → 4.0s | 3.5는 덜 들어감, **4.0에서 복귀 후 바로 안정**(18차) |
| full1 | 2차선 출발(상자 옆 다른 구간) | 출발 차선 자동 판단 1차선 오판, 방식 1이 바깥 선을 잡아 오른쪽 최대 → **2차선도 방식 2**(`guide_*_v2`) |
| **full2** | 2차선 출발 | **전체 성공**: 2차선 41초 → 1차선 변경 → 1차선 → 2차선 복귀 → 2차선 31초 → 횡단보도 정지 |
| **full3** | 1차선 출발 | **전체 성공**: 1차선 52초 → 2차선 복귀 → 2차선 31초 → 횡단보도 정지 |
| light1/2 | 신호등 확인(장애물 회피 끔) | **신호등 감지 검증 완료**: 빨간불에서 `light_stop` → `decision_auto` "STOP SIGNAL ACTIVE"로 정지, 신호가 바뀌면 해제되어 재출발(도경윤 "잘 된다" 확인). full2에서도 190.6초에 빨간불 감지. light1은 디스크 가득으로 깨짐. 1차선 직진에서 점선을 못 봐 왼쪽으로 흐름 → 원인: 밤 조명에서 점선 채도 22 → `s_max` 30. 이후 신호등 근처에서 화면이 붉게 물들어 다시 검출 실패(미해결) |

---

## 5. 현재 상태 (10/18 00시 무렵)

- 모든 ROS 노드 종료. 이 문서와 코드 변경을 `gyungyoon`에 **커밋**(push는 안 함). `origin/ganghee`의 DBSCAN `obs_evade`(10/16 7-8)와는 같은 파일이라 합칠 때 충돌함 — 팀 상의 필요.
- 남아 있는 bag: `bag_obs10~18`, `bag_full1~3`, `bag_light1(깨짐)/2`. 분석 스크립트는 `~/fmtc_session_1017/tools/`(`run_ana.py` 경과 표, `montage3.py` 검출 그림, `cmp.py`/`l2cmp.py` 방식 1·2 비교 등).
- 시스템: 자동 업데이트 정지, 전방 카메라 `exposure_dynamic_framerate=0` (재부팅 시 원래대로).

---

## 6. 성능 고도화를 위해 해야 할 일 (우선순위)

1. **조명에 강한 흰 선 판별** — 지금 조건(밝기 ≥130, 채도 ≤30)은 조명 색에 약함. 밤에는 흰색이 푸르스름하게(채도 22), 신호등 빨간불 근처에서는 붉게 찍혀 점선이 통째로 빠짐. 후보: 화면 전체 색 기준 보정(회색 세계 화이트밸런스) 후 판별, 고정 임계 대신 주변 바닥보다 얼마나 밝은지(국소 대비, top-hat), 카메라 화이트밸런스 고정(`white_balance_automatic=0`). 낮·밤·신호등 근처 bag을 모아 오프라인 검증부터.
2. **2차선 방식 2 기준값 재측정** — 지금 384/0.73은 방식 1과 맞춘 값. 차를 2차선 가운데에 두고 방식 2로 `calib_pose`처럼 직접 측정해 다시 정하기.
3. **차선 변경도 방식 2로** — 지금은 방식 1이라 "이전 선 사라짐/새 선 찾음"이 점선 빈 구간·주차선에 흔들림(6·7·8·15차). 방식 2의 선 종류(실선/점선)와 위치를 쓰면 "새 차선의 선"을 확실히 판단할 수 있음. 바꾸면 `obs_evade` 시간 값 재조정 필요.
4. **차선 변경을 시간 대신 상태로** — 우회전 6초, 직진 4초는 이 속도(명령 0.7)·배터리 상태에 맞춘 값. 속도가 바뀌면 다시 맞춰야 함. 라이다/카메라로 "새 차선 안쪽으로 얼마나 들어갔는지"를 보고 끝내는 방식, 또는 오도메트리(엔코더) 기반 거리 사용.
5. **`sign_detect` 가볍게** — CPU YOLO가 1~3초씩 멈춤(온도 80~87°C). 입력 크기 축소(`imgsz` 320), 프레임 건너뛰기, 모델 경량화/OpenVINO. 해결 후 `require_signals: true` 복귀.
6. **출발 차선 자동 판단 보강** — 상자 옆 2차선 구간에서 실선이 짧게 재져 1차선으로 오판. 방식 2의 선 종류 판단(긴 실선 vs 짧은 토막 여러 개)으로 바꾸기.
7. **1차선 조향 튐 다듬기** — 점선 토막이 바뀌는 순간 조향이 순간 최대까지 튐(1초 평균은 정상). `steer_filter_n` 3~5 시험.
8. **Arduino A4(조향 각도 센서)·8번 선 고정** — 이틀 연속 접촉 불량. 납땜 또는 커넥터 고정.
9. **커밋·병합** — `ganghee`의 `obs_evade`(DBSCAN 감지)와 합치는 방향 팀 상의. 감지 쪽은 DBSCAN, 차선 변경 동작은 이 코드로 가는 안이 자연스러움.
10. (선택) 신호등 감지 거리 분석 — 정지·재출발은 검증됨. 더 다듬으려면 `bag_light2_2236`(전방 압축 영상)으로 신호등이 처음 잡힌 거리와 정지 기준(박스 15500px²)을 넘는 시점을 확인해 정지 위치를 조정.
