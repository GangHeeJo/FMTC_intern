# Field Notes — 2026-10-16 (도경윤 노트북 환경 구축과 실차 점검)

> 먼저 [10/11 인수인계 문서](../summary/handoff_2026-10-11.md)를 읽으세요. 이 문서는 그 이후 도경윤 노트북에서 같은 절차를 따라 하며 겪은 문제를 적은 상세 기록입니다.

반영 상태 표기: **커밋** = 저장소에 올라감 / **로컬** = 이 머신의 파일에만 있고 커밋 안 함 / **시스템** = 이 머신의 OS 설정 / **임시** = 실행 중에만 유효(노드 재시작 시 사라짐) / **미해결**

이 문서 자체도 아직 **로컬**(커밋 안 함)입니다.

---

## 0. 환경

| 항목 | 값 |
|---|---|
| 노트북 | ASUS Vivobook 16 X1607CA, Intel Core Ultra 5 225H (14코어: 0~11번 고성능 최대 6.1~6.2GHz, **12~13번 저전력 최대 2.5GHz**), RAM 15GB, GPU 없음(CUDA 없음) |
| OS | Ubuntu 22.04.5, 커널 `6.8.0-138-generic`, X11 세션(`WaylandEnable=false`) |
| 브랜치 | `gyungyoon` (로컬에서 `ganghee` 086a6a9까지 fast-forward한 상태. 작업은 `gyungyoon`에 커밋) |
| 초기 설정 | 인수인계 2절 ①~④ 완료: `NATIVE=1 bash scripts/vm_setup.sh`, 재부팅, `autosuspend=-1`, `dialout`/`video` 그룹, 패키지 7개 빌드 확인 |

강희 노트북(10/11)과 **다른 기종**입니다. 무선 칩 번호도 다릅니다(아래 2절).

---

## 1. 오늘 겪은 문제 총정리

| # | 문제 | 원인 | 조치 | 반영 |
|---|---|---|---|---|
| 1 | 자동 주행 속도/조이스틱 축이 Xbox 컨트롤러와 안 맞음 | yaml 기본값이 다른 컨트롤러 기준(`axis_steer: 0`, `axis_drive: 3`), `th_scale: 1000` | 인수인계 ⑤대로 `th_scale: 450`, `axis_steer: 3`, `axis_drive: 1`. 실수 커밋 방지로 `git update-index --skip-worktree s1_stack/config/s1_stack.yaml` | **로컬** |
| 2 | 블루투스/Wi-Fi 안 됨 | 이 노트북 칩이 Ubuntu 22.04의 커널/펌웨어보다 최신. BT `8087:0026`: `Unsupported Intel hw variant (29)`. Wi-Fi `8086:7740`: `iwlwifi-bz-a0-hr-a0-80~86.ucode` 펌웨어 없음(`linux-firmware` 2022년판) | 조치 안 함. 조이스틱은 유선, 인터넷은 iPhone USB 테더링 | **미해결**(주행에는 영향 없음) |
| 3 | 차선 카메라가 트랙 밖(방 바닥)을 봄 → `Lane lost` 100% | 위에서 측면을 보는 차선 카메라(`5C3E0D1F`)의 각도가 10/11과 달라짐 | 카메라 각도 재조정. 오른쪽 흰 경계선이 검출 구역(x>330, 빨간 대각선 위)을 지나가게 맞춤 | **하드웨어** |
| 4 | 흰 경계선이 보이는데도 검출 0% | 영상이 어두워서 흰 선 밝기(HLS L)가 **78~89**로 `color_filter` 임계값(**L≥100**) 미만. 차체만 흰색으로 잡힘. `usb_cam`이 `brightness: 50`(카메라 기본값 128)으로 설정하고, 저녁이라 경기장 조명도 어두움 | `v4l2-ctl -d /dev/cam_lane -c brightness=128` → 검출 0% → 78% (90일 때는 16%) | **임시** (영구 반영은 아래 3절 결정 필요) |
| 5 | 카메라 프레임이 어두운 곳에서 줄어듦 | C920의 `exposure_dynamic_framerate=1`(어두우면 노출을 늘리려고 fps를 낮춤) | `v4l2-ctl -d /dev/cam_lane -c exposure_dynamic_framerate=0` | **임시** |
| 6 | 10/11 기준값(531, 0.524)으로는 조향이 -23°로 쏠림 | 카메라 각도를 바꿨으므로 기준값도 바뀜 | 차를 오른쪽 차선 가운데·직선과 나란하게 두고 `calib_pose.py`: **`guide_slope:=0.709 guide_x:=384`** (검출 100%, 표준편차 기울기 0.002 / 위치 0.4px). 적용 후 검출 98%, 조향 -4.5°~+2.8° | **실행 명령에만** (아래 4절) |
| 7 | 자동 모드에서 `STALE INPUT - STOPPING`이 자주 뜸 (판단의 13~50%) → A를 누르면 가다 서다 반복할 것 | `decision_auto`는 `/cmd_lane`이 0.5초, 신호가 1.0초 넘게 안 오면 정지. 아래 7-1, 7-2 두 원인이 겹침 | 7-1은 우회, 7-2는 **미해결** | 아래 |
| 7-1 | `sign_detect`(YOLO, CPU 추론)가 CPU를 **730~860%** 사용 | GPU가 없어 torch가 CPU 코어를 거의 다 씀 | `OMP_NUM_THREADS=4`만으로는 효과 없음(여전히 731%). `taskset -c 10-13`은 저전력 코어 12~13 포함이라 너무 느려져 신호 토픽이 1초 넘게 끊김(악화). **`OMP_NUM_THREADS=6 taskset -c 6-11`**(고성능 코어 6개)로 `/light_stop`·`/cross_stop` 6.7Hz, 최대 간격 0.19초로 안정 | **임시** (실행 명령에만) |
| 7-2 | `lane_trace`가 영상을 제때 못 받음: 카메라는 26~30Hz(`camera_info` 기준)인데 `/cmd_lane`은 **4.9~15Hz**, 최대 간격 **0.6~2.1초** | `lane_trace` CPU는 3~19%로 여유 → 연산이 아니라 **영상 전달 중 손실**로 보임. 구독 QoS가 `BEST_EFFORT`이고 영상이 장당 약 0.9MB(640×480 rgb8). FastDDS에서 큰 메시지가 best-effort로 조각나 빠지는 것으로 추정 (**미확인**) | 아래 "7-2 확인" 행과 6절 | **해결**(로컬, 6절) |
| 8 | C920이 3대 꽂혀 있음 | 시리얼 `555E0B1F`는 udev 규칙에 없는 여분 카메라 | 쓰지 않음. 허브 대역폭을 나눠 쓰므로 빼는 것을 권장 | — |
| 9 | 측정 도구 주의: `ros2 topic hz /cam_lane/image_raw`가 1~7Hz로 나옴 | 측정 도구(Python)가 큰 영상을 받느라 느린 것. 실제 카메라 주기와 다름 | 카메라 주기는 `/cam_lane/camera_info`로 잴 것 | — |
| 10 | 시험 중 `/joy`가 두 군데서 발행됨 | 버튼 확인용으로 띄운 `joy_node`가 남아 있었음 | 종료. `ros2 node list \| sort \| uniq -c`로 모두 1인지 확인 | — |
| 7-2 확인 | **해상도 시험으로 원인 확정**: 영상 전달 손실은 메시지 크기 때문 | 아무 처리 없는 구독기로 `/cam_lane/image_raw` 측정. **640×480(0.92MB/장): 카메라 26Hz인데 수신 3.2Hz, 최대 간격 4.94초.** **320×240(0.23MB/장): 수신 26.9Hz, 최대 간격 0.07초, 끊김 0.** `/cmd_lane`도 28.5Hz로 끊김 0, `STALE` 0 | 320×240에서는 `lane_trace`가 차선을 100% 못 찾음 — ROI·대각선·`guide_x` 등이 **640×480 좌표로 하드코딩**돼 있기 때문. 그래서 시험 후 640×480으로 되돌림. → 오후에 5절 후보 (a)로 해결: 차선 카메라 320×240 + `lane_trace`에서 640×480으로 리사이즈. 메인 스택 60초 측정에서 `STALE` 0/600 | **해결**(로컬, 6절) |
| 12 | 카메라 노드를 640×480으로 되돌리는 순간 카메라 허브가 끊김 (`usb 3-4-port1: disabled by hub (EMI?), re-enabling...` 두 번, 3-4.1 아래 카메라 전부 disconnect) | 2단 허브 부하로 장치가 한꺼번에 빠지는 현상(인수인계 1절에 언급). 카메라 스트림을 다시 여는 순간 발생 | 재연결 후 `/dev/cam_lane`은 다시 생겼지만 **`/dev/cam_front`(`D974F03F`)는 돌아오지 않음**. `usb_cam` 두 노드 모두 죽음(`Invalid v4l2 format`, `Cannot open device`). Arduino는 허브 1에 직접 꽂혀 있어 영향 없음 | **해결** — 20:35에 `/dev/cam_front` 다시 잡힘. 이후 메인 스택 재시작해 두 카메라 정상 |
| 11 | 노트북 화면이 계속 어두워짐. 그대로 두면 **자동 절전(suspend)으로 노드가 전부 멈출 위험** | GNOME 기본값: 5분 무입력 시 화면 끄기(`idle-delay 300`), 무입력 시 어둡게(`idle-dim`), 주변광 자동 밝기(`ambient-enabled`), 무입력 시 절전(`sleep-inactive-ac/battery-type 'suspend'`) | `gsettings set org.gnome.desktop.session idle-delay 0`<br>`gsettings set org.gnome.settings-daemon.plugins.power idle-dim false`<br>`gsettings set org.gnome.settings-daemon.plugins.power ambient-enabled false`<br>`gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'`<br>`gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type 'nothing'` | **시스템**(개인 설정) |

---

## 2. 확인한 것 (정상)

| 항목 | 결과 |
|---|---|
| 장치 연결 | `/dev/arduino_mega`→`ttyACM0`, `/dev/cam_lane`→`video8`(`5C3E0D1F`, 위에서 측면), `/dev/cam_front`→`video4`(`D974F03F`, 정면), `/dev/input/js0` 모두 생김. 라이다는 안 꽂음 |
| 조이스틱 매핑 (Xbox Series X, 유선) | A=`button[0]`, B=`button[1]`, Y=`button[3]`, 왼쪽 스틱 상하=`axis[1]`(위 +), 오른쪽 스틱 좌우=`axis[3]`(왼쪽 +). yaml과 일치. 수동(B) 모드 조향 방향은 아직 따로 확인 안 함 |
| 자동 모드 조향 방향 | 1차 자동 시험에서 `cmd_out` 조향 **-1000일 때 차가 오른쪽**으로 감(관찰). `lane_trace`의 부호(+rad = 오른쪽, `angular.z = -rad×2500`)와 일치 → `lane_trace` 경로의 부호는 정상 |
| 메인 스택 | 노드 7개 + `lane_trace` + `sign_detect` 각 1개. `serial_sender`가 Arduino를 엶. 시작 모드 STOP |
| 수동 모드 동작 특성 | 전후진은 비례가 아니라 **임계값(0.10)만 넘으면 최고 속도**(`th_scale`). 수동 운전 시 스틱을 짧게 |

---

## 3. 결정이 필요한 것

1. **차선 카메라 밝기 영구 반영**: `s1_stack.yaml`의 카메라 파라미터에 `brightness: 128`을 넣을지. 경기장 조명/시간대에 따라 달라지고 팀 전체 설정이라 상의 필요. 근본적으로는 `color_filter`의 L 임계값(100)을 파라미터로 열거나, 고정 임계값 대신 적응형으로 바꾸는 방법도 있음. → 오후에 `l_min`/`s_max` 파라미터로 엶(6절). 단 brightness 128에서는 `L≥100`만으로는 녹색 잔디까지 잡히므로, 128을 영구 반영한다면 `l_min:=130 s_max:=20`도 같이 써야 함.
2. **`sign_detect` CPU 제한 방식**: 지금은 `taskset`으로 우회. 코드에서 추론 주기를 낮추거나(신호등은 초당 몇 번이면 충분) 입력 해상도(`imgsz`)를 줄이는 방법이 더 근본적.
3. ~~**`lane_timeout_s`(0.5초)**: 7-2가 해결되지 않으면 1.0초로 늘리는 것도 방법.~~ → 7-2 해결로 필요 없음(0.5초 유지).
4. **정면 카메라 해상도**: 차선 카메라만 320×240으로 바꿈. 정면(`sign_detect`)은 640×480 유지 중이고 신호 토픽은 끊김 없음(9.2Hz, 최대 간격 0.13초). 320×240에서 YOLO 검출이 되는지는 미확인.
5. **오후에 연 파라미터의 기본값**: 새 파라미터 기본값은 모두 기존 동작과 같게 둠(팀원 환경 유지). 6절의 잘 된 값을 기본값으로 바꿀지는 팀 상의 필요.

---

## 4. 오늘 실행한 명령 (재현용)

```bash
# 터미널 1
ros2 launch s1_stack s1_stack.launch.py
# (켜진 뒤, 임시 카메라 설정 — usb_cam을 다시 켜면 사라지므로 매번)
v4l2-ctl -d /dev/cam_lane -c brightness=128
v4l2-ctl -d /dev/cam_lane -c exposure_dynamic_framerate=0

# 터미널 2 (오후 6차 시험에서 커브 완주한 설정. 오전 값은 linear_speed:=1.0 guide_slope:=0.709 guide_x:=384)
ros2 run lane_trace lane_trace --ros-args -p linear_speed:=0.7 -p lost_hold_s:=1.5 \
  -p guide_slope:=0.70 -p guide_x:=404 -p min_detect_ratio:=0.5 -p detect_window_s:=0.5 \
  -p l_min:=130 -p s_max:=20 -p right1:=270 -p b_rad:=0.004

# 터미널 3 (고성능 코어 6개로 제한)
OMP_NUM_THREADS=6 taskset -c 6-11 ros2 run sign_detect sign_detect --ros-args -p model_path:=$HOME/FMTC_intern/sign_detect/best_0808.pt
```

---

## 5. 다음에 할 일 (우선순위)

1. **7-2 영상 전달 손실 해결** (원인은 메시지 크기로 확정). 후보:
   - (a) 카메라 320×240 + `lane_trace` 안에서 640×480으로 `cv2.resize` 후 기존 로직 그대로 사용 → 코드 한 줄, 기준값은 `calib_pose.py`로 재측정. `sign_detect`(YOLO는 내부에서 640으로 리사이즈)도 320×240으로 충분할 가능성.
   - (b) 640×480 유지 + FastDDS 설정(XML 프로파일로 공유 메모리 세그먼트 크기 확대 등) → 코드 변경 없음, 모든 팀원 환경에 설정 필요.
   - (c) `lane_trace` ROI/기준을 해상도 비율로 계산하도록 리팩터링 (장기적으로 가장 깔끔).
   기준: `/cmd_lane` 최대 간격 0.5초 미만, `STALE` 0 근처.
   → **완료**: (a)로 해결(6절).
2. ~~`STALE` 비율이 0 근처가 된 뒤 **A로 자동 주행 첫 시험**~~ → **완료**: 6차 시험에서 커브 완주(6절).
3. B 모드로 **조향 방향** 확인 (반대면 `invert_steer: true`). 자동 경로 부호는 확인됨(2절), 수동은 미확인.
4. 인수인계 6절의 제어 쪽 과제: ~~조향 게인(`a_rad`, `b_rad`) 파라미터화~~(완료, 로컬), ~~조향/전진 멈칫 현상~~(7-2 해결로 사라짐: 6차 시험 34.7초 동안 정지 0번).
5. **남은 것** (오후 기준):
   - 직선 구간에서 `b_rad 0.004`로 좌우 흔들림이 없는지, 한 바퀴 전체 주행 확인.
   - 커브에서 경계선이 차체 쪽으로 붙으면 대각선 아래로 내려가 검출이 끊김(5차 시험). 게인을 올려 회피했지만 검출 구역 자체는 그대로.
   - 오늘 로컬 변경 커밋 여부와 새 파라미터 기본값 결정(3절 5).

---

## 6. 오후 작업 — 7-2 해결과 자동 주행 시험

### 6-1. 코드 변경 (모두 **로컬**, 커밋 안 함)

| 파일 | 변경 |
|---|---|
| `s1_stack/launch/s1_stack.launch.py` | 차선 카메라 `image_width/height` 640×480 → **320×240** (정면 카메라는 640×480 그대로) |
| `lane_trace/lane_trace/lane_trace.py` | ① 입력이 640×480이 아니면 640×480으로 `cv2.resize` 후 기존 로직 사용<br>② `min_detect_ratio`, `detect_window_s`: 최근 창 안의 검출 프레임 비율이 기준 미만이면 속도 0 (기본 0 = 끔)<br>③ `l_min`, `s_max`: `color_filter`의 HLS 밝기 하한/채도 상한 (기본 100/255 = 기존)<br>④ `a_rad`, `b_rad`: 조향 게인 ROS 파라미터화 (기본 0.5/0.0027 = 기존) |
| `scripts/lane_tuning/calib_pose.py` | 640×480 리사이즈, `--l_min`/`--s_max` 옵션 |
| `scripts/lane_tuning/bag_param_sweep.py` | 640×480 리사이즈 (없으면 320×240 bag에서 검출 0%로 나옴) |

symlink-install이라 재빌드 불필요.

### 6-2. 측정 결과

- 320×240 + 리사이즈: `lane_trace`만 띄워 30초 측정 시 `/cmd_lane` 30.1Hz, 최대 간격 0.041초. 메인 스택 + `sign_detect` 포함 60초: `/cmd_lane` 29.9Hz / 최대 0.070초, `/light_stop`·`/cross_stop` 9.2Hz / 최대 0.133초, **`STALE` 0/600**.
- 320×240 기준값 재측정(카메라 위치 조정 후, 차를 차선 가운데에): **`guide_slope 0.70`, `guide_x 404`** (검출 100%, 위치 표준편차 약 5px — 업스케일로 640 때 0.4px보다 큼).

### 6-3. 자동 주행 시험 (좌회전 커브, 모두 Y/B로 종료)

| 차수 | 설정 변경 | 결과 | 원인 / 조치 |
|---|---|---|---|
| 1 | 속도 1.0 | 12초 정상 후 커브에서 검출률 급락, **조향 +22.9°(오른쪽 최대) 고정인데 속도 1.0 유지**하며 우회전 | 띄엄띄엄 검출이 `lost_hold_s` 타이머를 계속 갱신해 정지 안 함 → ② 검출 비율 정지 추가. 1차 로그로 계산해 창 0.5초/기준 0.5 선택(정상 구간 최저 0.60, 문제 구간에서 31초 포화보다 약 2초 먼저 정지) |
| 2 | + `min_detect_ratio 0.5` | 정지 조건은 동작(오검출 구간에서 대부분 속도 0). 커브에서 검출 16~56%로 **가다 서다** | 영상 확인: brightness 128에서 **녹색 잔디(L 108~125, S 35~40)도 `L≥100` 통과** → 잔디/카펫 경계를 선으로 잡음. 흰 선은 L 135~194, S 0~7 → ③ `l_min 130`, `s_max 20`. bag 재생으로 오검출 144 → 0 확인 |
| 3 | (녹화만, 정지 상태) | 차가 커브 안쪽에서 경계선에 붙어 있어 검출 0 → 출발 안 함 | 직선 시작 위치로 옮김 |
| 4 | + `l_min 130 s_max 20`, 속도 0.7 | 2초 주행 후 커브 입구에서 정지, 재출발 안 함 | 마스크는 깨끗한데, 휜 선의 Hough 조각 중심점이 `right1`(330)보다 왼쪽이라 버려짐. `bag_param_sweep`: `right1` 330 → 270에서 커브 검출 25~38% → 82~98%, 포화 0%, 직선 99% 유지. 오버레이로 차체가 아닌 경계선을 잡는 것 확인 |
| 5 | + `right1 270` | **12.5초 연속 커브 주행**(검출 80~100%) 후 정지 | 조향이 -6° → -13°로 커졌지만 차가 바깥 경계선 쪽으로 밀려 선이 대각선 아래로 내려감(조향 부족으로 추정) → ④ 게인 파라미터화 |
| 6 | + `b_rad 0.004` | **34.7초 동안 정지 0번, 커브 완주.** 검출 86~100%(35~36초만 60%대), 조향 최대 약 -14°, 포화 1/1041프레임, `STALE` 0 | — |

bag(제어 토픽 + 320×240 차선 영상)은 이 노트북 `/tmp/claude-1000/.../scratchpad/bag_auto*`에 임시 저장(재부팅 시 삭제됨).

---

## 7. 저녁 작업 — 장애물 차선 변경(10/16 20:43 ~ 10/17 00:00 무렵)

> **내일 이어서 할 사람은 7-6(현재 상태)과 7-7(다음 할 일)부터 보세요.** 모든 변경은 **로컬**(커밋 안 함).

### 7-1. 시나리오 (도경윤 설명)

- 평소 **2차선**(오른쪽)을 달림. 차선 카메라는 차 **오른쪽**을 봄 → 2차선에서는 오른쪽 **실선**, 1차선에서는 오른쪽 **가운데 점선**을 따라감.
- 코스에는 장애물이 **2차선에 하나, 그 뒤 1차선에 하나**. 2차선 장애물 → 1차선으로, 1차선 장애물 → 2차선으로.
- 시험 구간은 좌회전 커브. 2차선 오른쪽 옆과 1차선 점선 오른쪽에 **주차 구역**(가는 흰 선 + 주차된 흰 차)이 있음.

### 7-2. 하드웨어/환경 사건 (순서대로)

| 시각 | 사건 | 원인/조치 |
|---|---|---|
| 21:11 | 라이다를 허브 3단에 꽂고 약 1분 뒤 **USB 허브 전체 끊김**(`usb3-port4: disabled by hub (EMI?)`). 카메라·Arduino·라이다 모두 빠짐 | 노트북 **포트 4번이 이후 어떤 장치도 인식 못 함**(`unable to enumerate`). 허브를 포트 3번으로 옮겨 해결 |
| 21:18~21:30 | Arduino가 허브·노트북 어디서도 `error -62`로 인식 안 됨 | **Arduino의 A4(조향 포텐셔미터 가운데 선, 초록선)와 8번(조향 PWM) 선이 빠져 있었음.** 다시 꽂으니 인식됨. A4가 비어 있으면 보정값이 떠다님(519/405/253) → 꽂은 뒤 정상(797/661, 가운데 724) |
| 21:33 | 노트북 **절전**(로그: `Lid closed`) | 덮개 닫힘 절전 설정(`lid-close-*-action 'suspend'`) 그대로임. **미조치** — 주행 중 덮개 닫히면 노드 정지 |
| — | 포트 부족: 1번 충전기, 2번 iPhone, 3번 허브, 4번 고장 | **조이스틱은 허브 1단 3번 칸**(`3-3.3`), Arduino 허브 1단 2번 칸, 라이다 허브 3단(`3-3.1.4.2`, 이후 문제없음) |
| 23:25 | 조이스틱으로 차가 안 움직임 | **배터리가 없었음.** 교체 후 Arduino 재보정(포트를 열면 재부팅+보정, 바퀴가 좌우로 움직임) |

라이다 방향(시험으로 확인): **차 앞 = 라이다 -x, 차 오른쪽 = 라이다 +y**(180° 돌아 장착). 라이다 바로 앞을 물체가 가리면(0.21m) 전방이 안 보임.

### 7-3. `obs_evade` 변경 (원래 코드 → 지금)

원래(`HEAD`): 정면 1.0~1.15m·좌우 ±1m에 점 30개 이상이 6스캔 연속이면 **왼쪽 최대 4초** 한 번, 이후 `lane_change_flag`를 **영원히 True** → `lane_trace`가 영상을 좌우 반전. `obs_evade2.py`(setup.py에 미등록)는 장애물마다 좌/우 4초 토글.

실차 1차 시험(원래 코드): 감지는 됨, 4초 꺾기로 **트랙 밖 주차 구역까지 감**, 반전된 `lane_trace`가 계속 왼쪽 최대 조향 → 영상 반전 방식은 오른쪽 카메라에서 동작 안 함 확인.

지금 `obs_evade.py` (파라미터는 모두 ROS 파라미터):
- 상태: `LANE2 → TO_LANE1 → LANE1 → TO_LANE2 → LANE2`. `lane_change_flag` = 1차선(가는 중 포함)이면 True, `/lane_changing` = 변경 중이면 True.
- `mode`: `return`(1차선에서 옆 장애물이 지나가면 복귀) / `toggle`(장애물마다 반대 차선). **둘 다 1차선에서도 전방 감지**. 변경 직후 `rearm_delay_s`(2초) 감지 쉼. `return`에서 1차선 앞·2차선 옆 모두 막히면 정지.
- 감지: `roi_near/roi_far`(기본 1.0/1.15), `roi_half_width` **0.4**(원래 1.0 → 옆 차선 장애물 오검출 방지).
- 차선 변경 `change_mode`:
  - `camera`(기본): `change_steer[0]`로 꺾음 → `/lane_detected`로 **원래 선 사라짐(5프레임) → 새 선 잡힘(3프레임)** → 반대로 꺾어 **정면 복귀**: `/lane_slope`가 그 차선 기준 기울기(`straighten_slope_l1` 0.728 / `_l2` 0.70)의 허용 범위 안에 들어오거나 지나치면 `lane_trace`에 넘김. 시간 제한 `cam_max_s`, `straighten_max_s`.
  - `timed`: `change_steer`/`change_time_s` 단계대로(S자 실험용).
- 허용 범위: `straighten_tol`, 차선별 `straighten_tol_l1`/`_l2`(**마지막에 추가, 아직 실차 미시험**).

### 7-4. `lane_trace` 변경 (모두 기본값 = 기존 동작)

| 파라미터 | 오늘 쓴 값 | 이유 |
|---|---|---|
| `guide_x_l1`, `guide_slope_l1` | 400, 0.728 | 1차선 점선 기준값(1차선 가운데에 놓고 측정) |
| `right2`, `right2_l1` | 640, **440** | 1차선에서 점선 오른쪽 주차선 제외 |
| `min_detect_ratio_l1`, `lost_hold_s_l1` | 0, **4.0** | 점선 칸 사이 빈 구간(2~3.6초) 통과 |
| `open_k` | **9** | 가는 흰 선(주차 칸 선) 지움. 주차선 착각 구간 포화 17.8% → 0%, 2차선 커브 검출 94→93% |
| `roi_top_l1` | **100** | 1차선 안쪽 깊이 들어가면 점선이 화면 위쪽에 보임 |
| `hold_steer_max_deg_l1` | **12** | 빈 구간 유지 조향 상한(5는 커브에서 부족) |
| `a_rad_l1`, `b_rad_l1` | 0.5, **0.0055** | 1차선은 커브 안쪽이라 더 꺾어야 함 |
| `roi_top_change`, `min_slope_change` | **160, 0.1** | 차선 변경 중에만. 새 차선 선이 위쪽에 눕게 보임. 100은 너무 먼 경계선을 잡아 너무 일찍 "찾음" |
| `flip_on_lane_change` | false | 예전 반전 동작은 이걸로만 |

새 토픽: `/lane_detected`(Bool), `/lane_slope`(Float32, 못 찾으면 -1). 구독: `/lane_changing`.

`scripts/lane_tuning/bag_param_sweep.py`: 이제 `open_k`, `l_min` 등 `lane_detect` 인자 모두 `--fixed`/`--param`으로 가능(640×480 리사이즈 포함).

### 7-5. 장애물 시험 기록 (bag은 `~/fmtc_session_1016/`, 8.1GB. 이전 것은 `/tmp/.../scratchpad`로 재부팅 시 삭제)

| bag | 설정 | 결과 → 조치 |
|---|---|---|
| `bag_obs1_2151` | 원래 코드 | 감지 OK, 4초 왼쪽으로 차선 이탈, 반전 후 계속 왼쪽 → 반전 끔 |
| `bag_obs2_2158` | S자 5단계 각 1.5초 | 옆으로 0.25m만 비킴, 상자 0.15m 앞 정지 → 감지 1m에서 꺾기엔 공간 부족 |
| full1~4 | return, timed 2.5/5초 | 5초도 1차선 못 감(관찰: "더 돌았다가 정면") → **카메라 기준 변경** 도입. full3은 `lane_trace` 재시작을 안 해 `/lane_detected`가 없던 무효 시험 |
| `bag_toggle1_2235` | camera | 주차 칸 **가는 선**을 점선으로 착각 → `open_k 9` |
| `bag_toggle2_2239` | + open_k | 점선 잡음, 하지만 1차선 안쪽 깊이 → 점선이 ROI 위로 → `roi_top_l1 100`, 빈 구간 조향 상한 |
| `bag_toggle3_2245` | + roi_top_l1 | 방향 바로잡기 전에 놓침 → **정면 복귀 단계** 추가(도경윤 제안) |
| `bag_toggle4_2249` | + 정면 복귀(최대 3초) | 1.44→1.33만 → 최대 6초 |
| `bag_toggle5_2328` | 최대 6초 | **1차선 변경 완전 성공**, 39초 1차선 주행 후 커브에서 조향 부족 → `b_rad_l1 0.0055`, 상한 12° |
| `bag_toggle6_2333` | 장애물 2개 | 1→2차선 복귀 때 실선을 **늦게**(못) 찾음, 8초 우회전으로 바깥 경계 앞까지 → 변경 중 인식 완화 |
| `bag_toggle7_2338` | roi_top_change 100 | 너무 먼 경계선을 잡아 일찍 "찾음" → 1차선으로 되돌아감 → 160 |
| `bag_toggle8_2341` | roi 160, tol 0.3, max 8/8 | **장애물 2개 전체 순서 성공**(1차선 6.8+5.4초, 2차선 5.9+4.0초). 단 2차선은 기울기 0.41(기준 0.70)에서 넘겨, `lane_trace`가 납작한 선의 y=300 연장점을 멀리 잡아 **오른쪽 최대**(경계선 쪽) → 선 놓쳐 자동 정지 → `straighten_tol_l2` 추가(미시험) |

### 7-6. 현재 상태 (10/17 00시 무렵 종료)

- ROS 노드는 **모두 종료**함. 커밋 안 함(`git status`: 수정 5개 + 이 문서).
- 하드웨어: 허브는 노트북 포트 3번(포트 4번 불량 의심), 조이스틱 허브 1단 3번 칸, Arduino 선 A4·8 다시 꽂음(배선 고정 필요할 수 있음).
- 내일 실행 명령(전부):
```bash
# 터미널 1
ros2 launch s1_stack s1_stack.launch.py
v4l2-ctl -d /dev/cam_lane -c brightness=128; v4l2-ctl -d /dev/cam_lane -c exposure_dynamic_framerate=0
# 터미널 2
ros2 run lane_trace lane_trace --ros-args -p linear_speed:=0.7 -p lost_hold_s:=1.5 \
  -p guide_slope:=0.70 -p guide_x:=404 -p min_detect_ratio:=0.5 -p detect_window_s:=0.5 \
  -p l_min:=130 -p s_max:=20 -p right1:=270 -p b_rad:=0.004 -p open_k:=9 \
  -p guide_x_l1:=400.0 -p guide_slope_l1:=0.728 -p right2_l1:=440 -p min_detect_ratio_l1:=0.0 \
  -p lost_hold_s_l1:=4.0 -p roi_top_l1:=100 -p hold_steer_max_deg_l1:=12.0 -p b_rad_l1:=0.0055 \
  -p roi_top_change:=160 -p min_slope_change:=0.1
# 터미널 3
OMP_NUM_THREADS=6 taskset -c 6-11 ros2 run sign_detect sign_detect --ros-args -p model_path:=$HOME/FMTC_intern/sign_detect/best_0808.pt
# 터미널 4 (장애물 시험 때마다 새로: 시작할 때 장애물이 감지 구역 안에 있으면 안 됨)
ros2 run obs_evade obs_evade --ros-args -p mode:=toggle -p roi_near:=1.3 -p roi_far:=1.45 \
  -p cam_max_s:=8.0 -p straighten_max_s:=8.0 -p straighten_tol_l1:=0.3 -p straighten_tol_l2:=0.12
```
- **주의**: 코드를 고치면 실행 중인 노드를 반드시 다시 띄울 것(full3 무효 시험 원인). 시험 후 `obs_evade`를 끄지 않으면 `lane_change_flag`를 계속 보내 다음 시험을 망침 → 끈 뒤 `ros2 topic pub --once /lane_change_flag std_msgs/msg/Bool "{data: false}"`.

### 7-7. 다음에 할 일

1. **`straighten_tol_l2:=0.12`로 장애물 2개 시험** — 2차선 복귀 후 `lane_trace`가 왼쪽으로 정렬하며 실선을 따라가는지. 정면 복귀가 8초 안에 안 끝나면 `straighten_max_s` 조정.
2. (근본) `lane_trace`의 위치 오차를 y=300 연장점 대신 **검출된 선분 근처**에서 재는 방식 검토 — 선이 납작하거나 세로일 때 조향 부호가 뒤집히는 문제.
3. `mode:=return` 실차 시험(옆 통과 판단 `side_*`는 가짜 신호로만 검증).
4. 신호등 시험(`sign_detect`: 빨강/노랑 박스 면적 15500px² 이상이어야 정지 — 가까워야 걸림).
5. 그 다음 원래 후보: 카메라 v4l2 설정을 launch에 넣기, `sign_detect` CPU 줄이기, 노트북 덮개 절전 끄기, 포트 4번 점검.
6. 오늘 변경 커밋 여부와 새 파라미터 기본값 팀 상의(민준: `obs_evade` 원래 의도 확인).

### 7-8. 참고: 같은 시간 강희 `origin/ganghee`에 올라온 것 (확인만, 합치지 않음)

`86933d5` `obs_evade`를 ROS1 보고서 버전(DBSCAN)으로 포팅, `ebf7f40` `s1_stack.launch.py`에 `obs_evade` 추가, `cca291e` 확인사항 주석. `vm_setup.sh`에 `scikit-learn` 추가.
→ 우리 로컬 `obs_evade.py`·`s1_stack.launch.py`와 **같은 파일을 고쳐 충돌**함. 강희 버전은 한 방향 꺾기(시간 공식) 후 `lane_change_flag`만 바꾸는 방식(정면 복귀·점선 추종 없음, 실차 미검증). 이 노트북엔 `sklearn` 미설치. 합칠 때 launch에 `obs_evade`가 들어가면 따로 띄우는 우리 `obs_evade`와 **두 개가 동시에** 뜨므로 주의. 팀과 어느 쪽으로 갈지 상의 필요.
비교 결과(코드 읽기만, 실행 안 함): 강희 버전에서 가져올 만한 것은 **감지 쪽** — DBSCAN 군집 + 점 개수 상·하한(50~200개)으로 벽·주차 차량 같은 큰 물체와 잡음을 거르고, 0.8~1.9m의 넓은 거리 창으로 일찍·안정적으로 감지, 장애물 좌우 위치(`y_coord`) 산출. **회피 동작**(한 방향 0.8초 안팎 시간 꺾기, 최고 속도)은 이 차에서 오늘 확인한 필요 시간(꺾기 6~7초 + 정면 복귀 4~5초)과 맞지 않아 가져올 필요 없음. 내일 녹화된 `/scan`(bag_toggle*, bag_obs*)으로 두 감지 방식을 오프라인 비교해 볼 것.
**결정(도경윤, 10/17 00시 무렵):** 강희 버전은 **참고로만** 남기고, 장애물 회피는 **우리 코드(로컬 `obs_evade`)로 끝까지** 완성한다. 위 오프라인 비교는 하지 않음(필요해지면 그때).
