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
