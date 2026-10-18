# S1 자율주행차 (FMTC)

> `main`은 2026-10-18에 `gyungyoon` 브랜치의 10/17 실차 검증 결과(장애물 2개 차선 변경 완성, 신호등/횡단보도 정지)로 동기화됨. 최신 상태와 실행 명령은 [docs/summary/handoff_2026-10-17.md](docs/summary/handoff_2026-10-17.md) 한 장만 먼저 읽으세요. 아래 "빌드 & 실행"은 코드 구조 설명용이고, 실제 실행 명령(터미널 순서, 파라미터 값 등)은 그 문서에 있습니다.

서울대학교 FMTC에서 운영하는 1/10 스케일 자율주행 RC카 "S1"의 ROS2(humble) 소프트웨어 스택. 카메라(차선/표지판 인식), 라이다(장애물 회피), 조이스틱/아두이노 제어를 통합해 차선 추종 + 신호등/횡단보도 정지 + 장애물 회피 자율주행을 수행한다.

제3회 미래형자동차 자율주행 SW경진대회(서울대 "두돈반" 팀) 출품 코드가 원형이며, 당시 ROS1 버전은 저장소 루트의 결과보고서 PDF에 부록 코드로 남아있다. 지금 이 저장소는 ROS2로 재구현된 버전이고, 처음부터 `rclpy` 기반으로 작성되어 ROS1→ROS2 전환 작업은 필요 없다 (rosserial도 쓴 적 없이 자체 시리얼 프로토콜로 시작함).

## 담당 (2026-10 검증 기준)

| 담당자 | 파트 | 패키지 | 브랜치 |
|---|---|---|---|
| 조강희 | 카메라 | `lane_trace`, `sign_detect` | `ganghee` |
| 김민준 | 라이다 | `gap_follow`, `wall_follow`, `obs_evade`, `sllidar_ros2` | `minjune` |
| 도경윤 | 제어·통신 | `s1_stack`, `Arduino/S1` | `gyungyoon` |

각자 자기 브랜치에서 작업 후 `main`에 push. `s1_stack`의 `decision_auto.py`/`cmd_mux.py`/`s1_stack.yaml`/`s1_stack.launch.py`처럼 여럿이 겹치는 파일을 건드릴 땐 미리 공유하기. 실차로 테스트하는 날엔 `main`을 건드리지 않기(빌드 깨지면 그 자리에서 다 같이 막힘).

## 문서 작성 규칙

현장 검증/디버깅 내용은 날짜별로 아래 형식으로 남긴다 (`docs/summary/handoff_2026-10-11.md` + `docs/records/field_notes_2026-10-11.md` 참고):

| 위치 | 용도 |
|---|---|
| `docs/summary/handoff_<날짜>.md` | **팀원이 먼저 읽을 한 장 요약** — 연결 방법, 실행 명령, 현재 상태, 우선순위 |
| `docs/records/field_notes_<날짜>.md` | 상세 기록 — 겪은 문제·원인·조치, 커밋/로컬/시스템 반영 상태 |
| `docs/records/<주제>_guide.md` | 특정 기능(예: 차선 추종) 튜닝/구조 가이드. 날짜에 안 묶임 |

summary 문서 맨 앞에서 해당 records 문서로 링크하고, README 맨 위는 **가장 최근 summary 하나만** 가리키면 된다.

## 패키지 구조

```
src/
├── lane_trace/     차선 인식 (HLS 필터 + Canny + Hough) → /cmd_lane
├── sign_detect/    YOLOv8 신호등·횡단보도 인식           → /light_stop, /cross_stop
├── obs_evade/      라이다 기반 장애물 회피               → /cmd_obs, /lane_change_flag
├── gap_follow/     라이다 gap-following
├── wall_follow/    라이다 벽 추종
├── sllidar_ros2/   RPLidar 공식 드라이버 (Slamtec 벤더 코드, 그대로 포함)
├── s1_stack/       조이스틱 입력, 주행모드 전환, 최종 명령 병합, 아두이노 시리얼 송신
├── Arduino/S1/     모터·조향 서보 구동 펌웨어 (S1.ino)
└── udev/           라이다/아두이노 장치명 고정 규칙
```

## 데이터 흐름

```
/cam_lane  → lane_trace   → /cmd_lane  ─┐
/cam_front → sign_detect  → /light_stop, /cross_stop ─┤
/scan      → obs_evade    → /cmd_obs   ─┼→ decision_auto → /cmd_auto ┐
                                         │  (우선순위: 정지신호 > 장애물 > 차선)   │
/joy → joy_to_twist → /cmd_manual ──────┘                              ├→ cmd_mux → /cmd_out → serial_sender → Arduino(S1.ino)
                                                                        │   (조이스틱으로 MANUAL/AUTO/STOP 모드 전환)
```

- `decision_auto`: 신호등/횡단보도 정지 → 장애물 회피(0.2s 이내 수신 시) → 평상시 차선 추종, 순으로 `/cmd_auto` 하나로 병합(2026-10-18: 장애물 회피가 신호 정지보다 위에 있던 순서를 공식 설계 스펙대로 뒤집음).
- `cmd_mux`: 조이스틱 버튼(A=자동, B=수동, **Y=정지**, Xbox 컨트롤러 기준 `btn_stop: 3`)으로 모드 전환, 자동/수동 명령에 타임아웃 적용해 신호 끊기면 자동 정지(failsafe).
- `serial_sender`: `geometry_msgs/Twist`를 `"TH:<-1000..1000> STN:<-1000..1000>"` 텍스트 라인으로 변환해 시리얼로 전송. rosserial 아님, 자체 프로토콜.

## 패키지별 토픽 & 동작 상세

### 카메라

**`lane_trace`** (node: `lane_masking_node`)
- Sub: `/cam_lane/image_raw` (`sensor_msgs/Image`), `/lane_change_flag`·`/lane_changing` (`std_msgs/Bool`)
- Pub: `/cmd_lane` (`Twist`), `/lane_detected` (`Bool`), `/lane_slope`·`/lane_line_len` (`Float32`) — 뒤 셋은 `obs_evade`가 차선 변경 완료/출발 차선 판단에 씀
- 검출 방식 2개(`detector` 파라미터): 방식 1(기존, HLS 필터 → Canny → `HoughLinesP`), 방식 2(`lane_detect_v2`, 흰 덩어리의 굵기·길이·채움비·볼록도로 점선/실선/주차선/흰 차를 구분). 2·1차선 각각 `detector`/`detector_l1`로 따로 고를 수 있음. **항상 오른쪽(2차선 실선 또는 1차선 점선) 한쪽만 봄**, 좌측 차선 로직 없음.
- `lane_change_flag`가 True면 1차선 기준값(`guide_x_l1` 등)으로 전환해 같은 오른쪽 점선을 계속 따라감 — 과거처럼 이미지를 반전해서 보는 방식은 `flip_on_lane_change`(기본 꺼짐)로만 남아있음.
- 차선 미검출 프레임에는 `lost_hold_s`(차선별로 다를 수 있음) 동안 직전 조향값을 유지, 그 이상 못 보면 중앙으로 복귀. 최근 구간 검출률이 `min_detect_ratio` 미만이면 정지.
- 속도는 `linear_speed` 파라미터(기본 0.5), 조향은 rad→`angular.z`(-1000~1000 스케일)로 변환.

**`sign_detect`** (node: `sign_detect`)
- Sub: `/cam_front/image_raw` (`sensor_msgs/Image`)
- Pub: `cross_stop` → `/cross_stop`, `light_stop` → `/light_stop` (둘 다 `std_msgs/Bool`)
- YOLOv8(`best_0808.pt`)로 매 프레임 추론. 클래스: `0=cross-walk, 1=traffic-light-green(무시), 2=traffic-light-red, 3=traffic-light-yellow`.
- 횡단보도는 bbox 중심이 화면 가운데 10~90% 안에 있을 때만 유효로 침, 신호등은 bbox 크기(`min_bbox_size=15500`) 이상일 때만(= 충분히 가까워졌을 때만) 유효.
- 최근 5프레임 중 3프레임 이상 감지돼야 최종 정지 신호로 확정(노이즈 필터링).

### 라이다 / 장애물 회피

**`obs_evade`** (실제 실행 파일은 `obs_evade.py`. `obs_evade2.py`는 예전 개선판 초안, `setup.py`에 연결 안 돼있어 `ros2 run`으로 안 돌아감 — 지금 쓰는 상태머신 버전으로 대체됨)
- Sub: `/scan` (`LaserScan`), `/lane_detected`·`/lane_line_len` (lane_trace가 보냄), `/lane_slope`
- Pub: `/cmd_obs` (`Twist`), `/lane_change_flag`, `/lane_changing` (`Bool`)
- 상태머신(`LANE2`→`TO_LANE1`→`LANE1`→`TO_LANE2`→…)으로 동작. 전방 ROI(`roi_near`~`roi_far`, 좌우 `roi_half_width`)에 장애물이 연속 `min_scans`번 잡히면 반대 차선으로 변경 시작. 변경 완료는 시간이 아니라 **lane_trace가 보내는 `/lane_detected`로 "원래 선이 사라지고 새 선이 잡혔는지"를 확인**(`change_mode: camera`)해서 판단, 이후 `/lane_slope`가 목표 기울기에 들어오면 정면 복귀 완료로 보고 넘김.
- `mode` 파라미터로 `toggle`(장애물 만날 때마다 반대 차선, **2026-10-17 실차 검증된 설정**)과 `return`(장애물을 옆으로 지나치면 자동으로 원래 차선 복귀) 중 선택 — `return` 모드는 코드는 있지만 실차 미검증.
- 평소(차선 변경 중이 아닐 때)엔 `/cmd_obs`에 아무것도 발행하지 않음 → `decision_auto`가 차선 추종으로 자연히 돌아감.

**`gap_follow`** / **`wall_follow`**
- 둘 다 Sub `/scan`, **Pub `/cmd_auto`** (파라미터로 토픽 바꿀 수 있지만 기본값이 `decision_auto`가 쓰는 토픽과 동일!).
- `gap_follow`: 360도 스캔에서 안전 버블(차체 폭 기준) 제외 후 가장 넓은 gap의 가중 중심으로 조향, 속도는 `max_speed`의 50% 고정.
- `wall_follow`: 왼쪽 벽까지의 수직/45도 거리로 PID 오차 계산, 유효 스캔이 적거나 0.5s 이상 새 스캔이 없으면 안전 타이머가 강제로 정지 명령 발행.
- ⚠️ **둘 다 `decision_auto`와 같은 `/cmd_auto`를 쓰므로, `s1_stack.launch.py`로 돌릴 때 같이 켜면 둘 중 누가 쓰든 서로 덮어씀.** 지금 메인 launch 파일엔 이 둘이 포함 안 돼있어서 별도의 단독 모드(F1TENTH 랩 주행용?)로 보임 — `decision_auto`와 동시 실행하면 안 됨.

**`sllidar_ros2`**
- Pub만 함: `/scan` (`sensor_msgs/LaserScan`). 하드웨어(RPLidar)와 시리얼로 직접 통신하는 Slamtec 공식 드라이버.

### 제어 (`s1_stack`)

**`joy_to_twist`**: Sub `/joy` (`sensor_msgs/Joy`) → Pub `/cmd_manual` (`Twist`). 조이스틱 축을 선형 매핑(데드존/스케일 파라미터화), 20Hz 주기 발행.

**`cmd_mux`**: Sub `/joy`(모드 전환 버튼), `/cmd_manual`, `/cmd_auto` → Pub `/cmd_out` (`Twist`), `/drive_mode` (`UInt8`). MANUAL/AUTO/STOP 3모드, 모드 전환 시 0.2초 정지 유예, AUTO 모드에서 0.3초 이상 `/cmd_auto`가 안 오면 자동으로 MANUAL로 강제 전환.

**`decision_auto`**: 위 데이터 흐름 설명대로 `/cmd_lane`+`/light_stop`+`/cross_stop`+`/cmd_obs` → `/cmd_auto`.

**`serial_sender`**: Sub `/cmd_out` (`Twist`), `/calib/start` (`std_msgs/Empty`) → 시리얼로 `TH:/STN:` 텍스트 전송. 캘리브레이션 명령 수신 시 6초간 0,0 고정.

**`Arduino/S1/S1.ino`**: 시리얼로 `TH:/STN:` 라인을 받아 파싱, 모터 PWM/조향 서보 구동. 신호가 끊기면(타임아웃) 자체 failsafe로 정지.

## 빌드 & 실행

> 아래는 기본 빌드/실행 구조만 설명. **실제 차에서 돌릴 때 쓸 파라미터 값(특히 `lane_trace`/`obs_evade`의 `--params-file`)과 CPU 코어 배정(`taskset`)은 [10/17 인수인계](docs/summary/handoff_2026-10-17.md) 그대로 따라 하세요** — 기본값으로 그대로 실행하면 조향이 쏠리거나 속도가 너무 빠름.

```bash
# colcon 워크스페이스 루트(이 src/의 상위 폴더)에서
colcon build --symlink-install
source install/setup.bash

ros2 launch s1_stack s1_stack.launch.py
```
`s1_stack.launch.py`가 띄우는 것: `joy_node`, `joy_to_twist`, `cmd_mux`, `serial_sender`, `decision_auto`, `sllidar_node`, 카메라 2대(`cam_lane` 320x240, `cam_front` 640x480).

**`lane_trace`, `sign_detect`, `obs_evade`는 launch 파일에 없어서 각자 따로 실행해야 함** (CPU가 부족해서 코어를 나눠 쓰기 위함):
```bash
taskset -c 0-5 ros2 run lane_trace lane_trace --ros-args --params-file <튜닝 yaml>
taskset -c 0-5 ros2 run obs_evade obs_evade --ros-args --params-file <튜닝 yaml> -p start_lane:=2
OMP_NUM_THREADS=6 taskset -c 6-11 ros2 run sign_detect sign_detect --ros-args -p model_path:=/절대/경로/best_0808.pt
```
`sign_detect`의 `model_path`는 필수 파라미터(기본값 없음, 안 주면 바로 에러). 디버그용 OpenCV 창(`cv2.imshow`)을 보려면 `lane_trace`/`sign_detect`에 `-p debug_view:=true` 추가(기본 꺼짐, `obs_evade`는 영상 처리가 없어서 해당 없음).

## 알려진 이슈 (2026-10-17 실차 검증 기준)

- **밤 조명·신호등 근처에서 흰 선 검출이 깨짐**: 흰 선이 푸르스름(밤)하거나 붉게(신호등 근처) 찍히면서 색 기준(HLS `l_min`/`s_max`)을 벗어남. 낮에만 검증됨. 근본 해결은 화이트밸런스 보정 등 조명-불변 검출 방식 필요.
- **`sign_detect`(YOLO, CPU 추론)가 1~3초씩 멈춤**: CPU 온도 80도대까지 올라가며 멈춰서 `decision_auto`의 STALE 판정을 유발. 장애물 회피 시험 때는 `decision_auto`를 `require_signals:=false`로 띄워서 끊김 감지만 꺼서 우회 — **이 설정은 경기용으로 쓰면 안 됨**(신호 끊김에 대한 안전장치가 꺼짐). 근본 해결은 모델 경량화(`imgsz` 축소, OpenVINO 등).
- `obs_evade`의 출발 차선 자동 판단(`start_lane:=0`)이 특정 구간(상자 옆 2차선)에서 오판. 지금은 `start_lane`을 1/2로 직접 지정해서 씀.
- `obs_evade`의 `mode: return`(장애물을 옆으로 지나치면 자동 복귀)은 코드는 있지만 실차 미검증 — 검증된 건 `mode: toggle`(장애물 만날 때마다 반대 차선)뿐.
- 차선 변경 타이밍(`cam_min_s`/`cam_max_s`/`straighten_max_s` 등)은 특정 속도(`change_speed`)·배터리 상태에서 맞춘 경험값 — 속도가 바뀌면 재조정 필요.
- Arduino 조향 각도 센서(A4, 8번 선) 접촉 불량이 이틀 연속 발생 — 바퀴가 명령 없이 제멋대로 움직이면 이 선부터 확인 (하드웨어, 미해결).
- **DBSCAN 라이다 클러스터링 미구현**: 예전 ROS1 코드(결과보고서 PDF)엔 `sklearn.cluster.DBSCAN` 기반 장애물 클러스터링이 있었는데, 현재 `obs_evade`/`gap_follow`/`wall_follow`는 전부 단순 ROI/거리 임계값 방식임 (`ganghee` 브랜치에 DBSCAN 포팅판이 있으나 미병합). 김민준 담당 파트.
- **주차(parking) 기능 전체가 없음**: HW엔 주차용 초음파센서(HC-SR04 ×2)가 있는데 현재 repo엔 초음파 관련 노드/토픽이 아예 없음.

## 노트북(WSL2/VM) 환경 설정

`scripts/vm_setup.sh`가 아래 내용(locale, ROS2 설치, CycloneDDS, `setuptools` 핀, udev 규칙 등)을 대부분 자동으로 해줌. `NATIVE=1 bash scripts/vm_setup.sh`로 실행 (실차 노트북이면 USB 자동 절전도 같이 꺼줌).

- **colcon build가 "성공한 것처럼 보이는데" 실제로는 패키지가 등록 안 됨**: `pip install`(ultralytics/torch 등)이 최신 `setuptools`를 끌고 들어와 ROS2 humble의 `ament_python` 빌드 방식과 충돌. `ros2 pkg list`에 패키지가 안 뜨면 `pip install "setuptools==58.2.0"` 후 `rm -rf build install log && colcon build --symlink-install`로 클린 재빌드.
- **WSL2에서 노드끼리 서로 못 찾음**: 가상 네트워크가 기본 DDS(Fast-DDS)의 멀티캐스트 디스커버리를 막음 → `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`로 전환(스크립트가 처리).
- **WSL2/VirtualBox 둘 다에서 카메라 영상이 구독자한테 하나도 안 들어옴** — 2026-10-10·10-11에 둘 다 확인된 문제(QoS가 아니라 가상화 환경 자체의 한계로 보임). 영상이 큰 토픽(카메라)은 **네이티브 Linux에서만 정상 동작 확인됨**. 카메라를 WSL2/VM으로 테스트하지 말고 네이티브 Ubuntu로 넘어갈 것.
- USB 카메라/라이다/아두이노는 WSL2에선 `usbipd-win`(윈도우 쪽)으로 attach 필요 — 재부팅마다 다시 attach.
