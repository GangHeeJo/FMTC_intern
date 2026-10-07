# S1 자율주행차 (FMTC)

서울대학교 FMTC에서 운영하는 1/10 스케일 자율주행 RC카 "S1"의 ROS2(humble) 소프트웨어 스택. 카메라(차선/표지판 인식), 라이다(장애물 회피), 조이스틱/아두이노 제어를 통합해 차선 추종 + 신호등/횡단보도 정지 + 장애물 회피 자율주행을 수행한다.

제3회 미래형자동차 자율주행 SW경진대회(서울대 "두돈반" 팀) 출품 코드가 원형이며, 당시 ROS1 버전은 저장소 루트의 결과보고서 PDF에 부록 코드로 남아있다. 지금 이 저장소는 ROS2로 재구현된 버전이고, 처음부터 `rclpy` 기반으로 작성되어 ROS1→ROS2 전환 작업은 필요 없다 (rosserial도 쓴 적 없이 자체 시리얼 프로토콜로 시작함).

## 담당 (2026-10 검증 기준)

| 담당자 | 파트 | 패키지 | 브랜치 |
|---|---|---|---|
| 조강희 | 카메라 | `lane_trace`, `sign_detect` | `ganghee` |
| 김민준 | 라이다 | `gap_follow`, `wall_follow`, `obs_evade`, `sllidar_ros2` | `minjune` |
| 도경윤 | 제어·통신 | `s1_stack`, `Arduino/S1` | `gyungyoon` |

각자 자기 브랜치에서 작업 후 `main`에 push. `s1_stack`의 `decision_auto.py`/`cmd_mux.py`/`s1_stack.yaml`/`s1_stack.launch.py`처럼 여럿이 겹치는 파일을 건드릴 땐 미리 공유하기. 실차로 테스트하는 날엔 `main`을 건드리지 않기(빌드 깨지면 그 자리에서 다 같이 막힘).

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
                                         │  (우선순위: 장애물 > 정지신호 > 차선)   │
/joy → joy_to_twist → /cmd_manual ──────┘                              ├→ cmd_mux → /cmd_out → serial_sender → Arduino(S1.ino)
                                                                        │   (조이스틱으로 MANUAL/AUTO/STOP 모드 전환)
```

- `decision_auto`: 장애물 회피(0.2s 이내 수신 시 최우선) → 신호등/횡단보도 정지 → 평상시 차선 추종, 순으로 `/cmd_auto` 하나로 병합.
- `cmd_mux`: 조이스틱 버튼(A=자동, B=수동, X=정지)으로 모드 전환, 자동/수동 명령에 타임아웃 적용해 신호 끊기면 자동 정지(failsafe).
- `serial_sender`: `geometry_msgs/Twist`를 `"TH:<-1000..1000> STN:<-1000..1000>"` 텍스트 라인으로 변환해 시리얼로 전송. rosserial 아님, 자체 프로토콜.

## 패키지별 토픽 & 동작 상세

### 카메라

**`lane_trace`** (node: `lane_masking_node`)
- Sub: `/cam_lane/image_raw` (`sensor_msgs/Image`), `/lane_change_flag` (`std_msgs/Bool`)
- Pub: `/cmd_lane` (`geometry_msgs/Twist`)
- HLS 색공간에서 흰색만 필터링 → ROI 크롭 → auto-Canny → `HoughLinesP`로 직선 검출 → 기울기/위치로 필터링한 점들을 1차 함수로 피팅해 조향각 계산. **현재 오른쪽 차선만 봄** (ROI `right1=330~right2=640`), 좌측 로직 없음.
- `lane_change_flag`가 True면 (장애물 회피로 반대 차선으로 넘어간 상태) 이미지를 좌우 반전해서 같은 로직을 재사용하고, 조향값도 반전해서 발행.
- 차선 미검출 프레임에는 직전 조향값(`last_steer_rad`)을 그대로 유지해 발행 (뚝뚝 끊기는 조향 방지).
- 속도는 `linear.x = 0.5` 고정, 조향은 rad→`angular.z`(-1000~1000 스케일)로 변환.

**`sign_detect`** (node: `sign_detect`)
- Sub: `/cam_front/image_raw` (`sensor_msgs/Image`)
- Pub: `cross_stop` → `/cross_stop`, `light_stop` → `/light_stop` (둘 다 `std_msgs/Bool`)
- YOLOv8(`best_0808.pt`)로 매 프레임 추론. 클래스: `0=cross-walk, 1=traffic-light-green(무시), 2=traffic-light-red, 3=traffic-light-yellow`.
- 횡단보도는 bbox 중심이 화면 가운데 10~90% 안에 있을 때만 유효로 침, 신호등은 bbox 크기(`min_bbox_size=15500`) 이상일 때만(= 충분히 가까워졌을 때만) 유효.
- 최근 5프레임 중 3프레임 이상 감지돼야 최종 정지 신호로 확정(노이즈 필터링).

### 라이다 / 장애물 회피

**`obs_evade`** (실제 실행 파일은 `obs_evade.py`, entry point 기준. `obs_evade2.py`는 좌우 번갈아 회피하는 개선판 초안인데 아직 `setup.py`에 연결 안 돼있어서 `ros2 run`으로는 안 돌아감 — 둘 중 뭘 쓸지 확인 필요)
- Sub: `/scan` (`sensor_msgs/LaserScan`)
- Pub: `/cmd_obs` (`geometry_msgs/Twist`), `lane_change_flag` → `/lane_change_flag` (`std_msgs/Bool`)
- 전방 ROI(x: -1.0~-1.15m, y: ±1m)에 포인트가 30개 이상 잡히는 프레임이 5번 연속되면 장애물로 판단, **1회성으로 좌측 최대 조향 + 절반 속도를 1초간** `/cmd_obs`에 발행. 한 번 트리거되면 `evasion_triggered`가 다시 안 풀려서 그 이후로는 재판단을 안 함 (차량 한 사이클당 회피 1번만 가정).
- `lane_change_flag`는 트리거 이후 영원히 `True` 유지 → `lane_trace`가 이걸 보고 반대 차선 기준으로 계속 주행.

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

**`Arduino/S1.ino`**: 시리얼로 `TH:/STN:` 라인을 받아 파싱, 모터 PWM/조향 서보 구동. 신호가 끊기면(타임아웃) 자체 failsafe로 정지.

## 빌드 & 실행

```bash
# colcon 워크스페이스 루트(이 src/의 상위 폴더)에서
colcon build --symlink-install
source install/setup.bash

ros2 launch s1_stack s1_stack.launch.py
```

카메라 노드는 `s1_stack.launch.py`에 포함돼있지 않아 별도 실행 필요:
```bash
ros2 run lane_trace lane_trace
ros2 run sign_detect sign_detect --ros-args -p model_path:=/절대/경로/best_0808.pt
```
`sign_detect`의 `model_path`는 필수 파라미터(기본값 없음, 안 주면 바로 에러). 디버그용 OpenCV 창(`cv2.imshow`)을 보려면 두 노드 모두 `-p debug_view:=true` 추가 (기본 꺼짐 — 디스플레이 없는 SSH 환경에서 안전하게 돌리기 위함).

## 알려진 이슈 / 확인 필요 (2026-10 기준)

- `lane_trace`는 현재 **오른쪽 차선만** 검출함. 보고서 PDF의 예전 ROS1 버전은 좌/우 양쪽 차선 + `check_pixel_color` 색 검증까지 있었는데, ROS2로 옮기면서 빠짐 — 의도적 단순화인지 확인 필요.
- `usb_cam`, `cv_bridge`가 humble용으로 실제 설치돼있는지, `numpy<2.0` 고정이 유지되는지(README 참고) 검증 차량에서 확인 필요.
- `lane_trace`의 ROI/HLS 임계값은 특정 조도 기준 튜닝값 — 10월 검증 트랙(4층 로비) 조명에 맞게 재조정 필요할 수 있음.
