# Field Notes — 2026-10-11 (네이티브 Ubuntu 22.04 환경 구축)

10일 노트(`field_notes_2026-10-10.md`)의 결론대로 카메라 노드는 네이티브 Linux에서
돌려야 해서, 개발 머신을 Ubuntu 22.04.5(네이티브)로 옮겨 환경을 구축했다.

## 환경 구축

- `scripts/vm_setup.sh`가 VM뿐 아니라 네이티브 Ubuntu 22.04에서도 그대로 동작함.
  ROS2 humble, `usb_cam`/`cv_bridge`/`joy`/CycloneDDS, pip 패키지(ultralytics, torch,
  `setuptools==58.2.0`, `numpy<2.0`, pyserial), `.bashrc`, 빌드, udev 규칙 복사까지 오류 없이 끝남.
- 검증: 전체 패키지(`lane_trace`, `sign_detect`, `s1_stack`, `obs_evade`, `gap_follow`,
  `wall_follow`, `sllidar_ros2`, `usb_cam`) `ros2 pkg list` 등록 확인.
  `s1_stack.launch.py`는 하드웨어 없이도 기동됨.
- 설치 스크립트는 sudo 비밀번호 입력이 필요하므로 **일반 터미널에서** 실행해야 함
  (Claude Code의 `!` 명령은 tty가 없어 비밀번호를 못 받음).
- `dialout`/`video` 그룹은 **재로그인해야** 적용됨.
- `sllidar_ros2` 빌드 중 나오는 `onDecodingError` 관련 경고는 벤더 코드의 unused-parameter 경고라 무시해도 됨.
- 이 저장소에 git 사용자 정보가 없는 새 머신이면 `git config user.name/user.email`을 먼저 설정할 것.

## `STALE INPUT - STOPPING`이 계속 나오는 이유

`decision_auto`는 기본값(`require_signals:=true`)에서 입력 3개가 모두 신선해야 움직인다.

| 입력 | 허용 지연 | 발행 노드 |
|---|---|---|
| `/cmd_lane` | 0.5초 | `lane_trace` |
| `/light_stop` | 1.0초 | `sign_detect` |
| `/cross_stop` | 1.0초 | `sign_detect` |

하나라도 빠지면 `linear.x = 0`으로 정지한다. `/light_stop`·`/cross_stop`은
`/cam_front/image_raw`가 들어와야 발행되므로 전방 카메라(또는 `sign_detect`)가 없으면 계속 정지한다.

재현(하드웨어 없이 `/cmd_lane`만 10Hz 발행): 기본값은 6초 내내 STALE, `require_signals:=false`는
시작 직후 몇 번(첫 메시지 도착 전)만 나오고 이후 사라짐.

카메라 1대로 차선만 테스트할 때:

```bash
ros2 run s1_stack decision_auto --ros-args -p require_signals:=false
```

`s1_stack.launch.py`는 이 파라미터를 넘겨 주지 않는다.
`ros2 topic pub --once`처럼 한 번만 보낸 입력은 0.5초 뒤 stale이 되므로 `-r 10`으로 계속 발행해야 한다.

## 다음에 할 것

1. 재로그인 후 Arduino/라이다/C920 연결, udev 심볼릭 링크(`/dev/arduino_mega`, `/dev/rplidar`, `/dev/cam_lane`, `/dev/cam_front`) 확인.
2. `usb_cam`을 `-r __ns:=/cam_lane`으로 띄워 `ros2 topic hz /cam_lane/image_raw` 확인.
3. STALE이 계속되면 `bash scripts/s1_status.sh`로 끊긴 입력 확인. 어떤 입력을 넣었을 때 났는지 메모.
4. 10일 노트의 "아직 검증 안 됨" 항목(조향, `sign_detect`, 라이다 노드, 통합 launch) 진행.

## Arduino USB가 계속 끊기던 문제: 원인은 USB 자동 절전(autosuspend)

증상: Arduino(`/dev/arduino_mega`)가 몇 초마다 USB에서 끊겼다 붙었다 하며
`usb 3-3.4-port3: disabled by hub (EMI?), re-enabling...`이 반복됨. 이때 `serial_sender`는
첫 쓰기에서 `write failed: [Errno 5] Input/output error`로 죽음.

- 코드/ModemManager/케이블/배터리 쪽이 아니었음. 노드를 하나도 안 돌려도 끊겼고, 반대로
  launch로 카메라 스트리밍과 포트 사용 중일 때는 안정적이었음 → 장치가 놀 때 절전으로 들어가는 것이 단서.
- 확인: `usbcore.autosuspend = 2`(2초), 카메라와 안쪽 허브가 `runtime_status=suspended`.
- 해결: 자동 절전 끄기. 임시 적용은 아래, 영구 적용은 GRUB 부팅 옵션 `usbcore.autosuspend=-1`.

```bash
# 임시 (재부팅하면 사라짐)
sudo sh -c 'echo -1 > /sys/module/usbcore/parameters/autosuspend; for f in /sys/bus/usb/devices/*/power/control; do echo on > $f; done'
# 영구
sudo sed -i 's/^GRUB_CMDLINE_LINUX_DEFAULT="\(.*\)"/GRUB_CMDLINE_LINUX_DEFAULT="\1 usbcore.autosuspend=-1"/' /etc/default/grub
sudo update-grub
```

- 효과: 같은 허브 경유 상태에서 40초 동안 Arduino 재연결 11번 → 0번. 모터 구동 중에도 안 끊김.
- 새 머신 셋업(`scripts/vm_setup.sh`)에도 이 부팅 옵션을 넣는 것이 안전함.
- 참고: 끊김 진단 중 USB 허브, 케이블, 직결 여부를 많이 바꿨는데 결과적으로 절전이 주원인이었음.
  비슷한 증상이면 `dmesg`의 `disabled by hub (EMI?)`와 `/sys/bus/usb/devices/*/power/runtime_status`부터 확인할 것.

## `serial_sender`: 끊겨도 죽지 않고 자동 재연결

USB가 순간적으로 끊겨도 노드가 종료되지 않도록 수정(커밋 `4e1f625`). 끊기면 전송을 멈추고(정지 유지)
1초마다 재연결을 시도하며, 재연결 직후 Arduino 부팅 시간(2초)은 전송하지 않음. 이전 명령은 재전송하지 않음.
포트를 열면 Arduino가 리셋되므로 launch 직후 약 10초는 기다린 뒤 조작할 것.

## 조이스틱 (Xbox Series 컨트롤러, USB 유선)

- 버튼: 0=A(자동), 1=B(수동), 2=X, 3=**Y(정지)**. README에는 X가 정지로 적혀 있으나 설정(`btn_stop: 3`)은 Y임.
- 축: 왼쪽 스틱 상하=1, 좌우=0, 오른쪽 스틱 상하=4, 좌우=3. 기본 설정(`axis_drive: 3`)은 다른 컨트롤러 기준이라
  이 컨트롤러에서는 로컬에서 `axis_steer: 3`(오른쪽 스틱 좌우), `axis_drive: 1`(왼쪽 스틱 상하)로 바꿔 사용.
  이 변경은 컨트롤러 의존적이라 커밋하지 않음.
- `cmd_mux`는 STOP으로 시작하며 조이스틱 버튼을 눌러야 모드가 바뀜. 정지 후 스틱은 무시됨.

## 이 노트북(Samsung 940XHA, Lunar Lake)의 제약

- **블루투스와 Wi-Fi가 동작하지 않음**: 내장 칩(BT `8087:0037`, Wi-Fi `8086:a840`)에 커널 `6.8.0-138` 드라이버가
  붙지 않아 `hci0`/`wlan`이 없음. 인터넷은 iPhone USB 테더링으로 해결. 조이스틱은 유선(USB-C)만 가능.
- 포트가 부족함: USB-A 1개(허브), USB-C 2개(충전기/테더링/조이스틱 등 공유).
  충전 중인 USB-C 포트는 데이터 호스트로 동작하지 않음(`/sys/class/typec/port*/data_role`로 확인 가능).
  C→A 변환 어댑터가 인식되지 않는 경우가 있었으니(`partner=0`), 안 되면 다른 어댑터를 쓸 것.

## 테스트용 임시 설정 (로컬에만 있고 커밋하지 않음)

`s1_stack/config/s1_stack.yaml`: `th_scale: 200`(기본 1000, 저속 시험용), 조이스틱 축 번호.
커밋 전에 이 파일은 반드시 제외하거나 원래대로 되돌릴 것.

## 차선 추종(lane_trace) 튜닝 결과 (rosbag 기반)

`ros2 bag record`로 `/cmd_lane /cmd_auto /cmd_out /drive_mode /joy /cam_lane/image_raw`를 60초씩 기록하고,
저장된 영상에 `lane_detect`를 오프라인으로 다시 돌려 설정 후보를 비교했다. 현장에 권하기 전에 영상으로 먼저 검증하는 방식이 빨랐다.

**기준선(가장 잘 됐던 설정, 직선에서 조향이 0으로 수렴, 코너에서 올바른 방향 유지, 전진 끊김 거의 없음)**

```bash
ros2 launch s1_stack s1_stack.launch.py            # s1_stack.yaml의 th_scale: 450 (로컬 임시값)
ros2 run lane_trace lane_trace --ros-args -p linear_speed:=1.0 -p lost_hold_s:=1.5 -p guide_slope:=0.524 -p guide_x:=531
ros2 run sign_detect sign_detect --ros-args -p model_path:=$HOME/FMTC_intern/sign_detect/best_0808.pt
```

**원인과 수정(모두 파라미터이고 기본값은 기존 동작과 같음)**

- 조향이 한쪽으로 몰림: 기준값 `guide_slope=1.11`, `guide_x=550`이 예전 카메라 설치에 맞춘 값이라 지금 카메라에서는 이상적인 자세에서도
  조향이 +850 근처로 나왔다. 원하는 자세에 놓고 선의 기울기/위치를 5초 측정해 `guide_slope:=0.524 -p guide_x:=531`로 보정.
- 찔끔찔끔 전진: 차선 검출이 프레임마다 깜빡이면 검출 못 한 프레임에서 `linear.x=0`. `lost_hold_s`(놓친 뒤 전진과 마지막 조향 유지 시간)로 완화.
  `lost_hold_s>0`이면 그 시간을 넘겨 놓칠 때 조향도 중앙으로 복귀.
- 정수로 준 파라미터(`-p guide_x:=531`)가 `InvalidParameterTypeException`: 동적 타입 허용으로 수정.
- 추가한 파라미터: `linear_speed`, `lost_hold_s`, `guide_x`, `guide_slope`, `min_slope`, `right1`, `roi_top`, `steer_filter_n`.

**하지 말아야 할 것(영상 재생으로 확인)**

| 변경 | 결과 |
|---|---|
| `min_slope 0.1`, `right1 200` | 주차 박스 가로선을 차선으로 잡아 조향이 ±1000으로 진동 (포화 10% → 24%) |
| `roi_top 280` (가까이만 보기) | 직선에서는 포화 0%지만 위치가 다르면 검출률 2~28%로 붕괴 |
| `th_scale 600` | 조향 능력을 넘어 트랙 밖으로 벗어남. 450이 한계로 보임 |

**조향이 한쪽으로 몰리는 구간에서 한 번에 하나씩만 바꿀 후보**: `min_slope:=0.35`. 차가 트랙 중앙 쪽으로 가서 선이 완만하게 보일 때(검출률 34% → 73%)
도움이 되지만 잘 되던 주행에서는 포화가 4.8% → 11.3%로 늘어 맞교환임. 현장 검증은 아직 못 함.
중앙값 필터(`steer_filter_n:=5`)는 영상 재생에서 조향 튐 5.6% → 2.0%였으나 현장 검증은 못 함.

**코너(검출 약 26%)**: 오른쪽 경계선만 보는 구조라 코너에서 선이 화면 밖으로 휘어 빠진다. 근본 해결은 왼쪽 점선(중앙선)도 같이 쓰는 것(미구현).

**조향/전진이 번갈아 멈칫하는 현상**: 명령(`/cmd_out`)은 끊기지 않았음(40초 전진 100%). 조향 모터(PWM 150, 목표 ±3~4°에서 on/off)와 구동 모터(`th_scale 450` → PWM 81/255) 전력 간섭 가능성.
수동 모드에서 조향 없이 전진할 때와 조향하며 전진할 때를 비교해 확인하는 것이 아직 안 됨.

**교훈**

- 시험용 노드를 띄우면 PID로 확실히 종료할 것. 시험 노드(`lane_ptest`)가 남아 `/cmd_lane`에 같이 발행해 몇 개 기록이 오염됐다.
  기록 전에는 `ros2 node list | sort | uniq -c`로 중복 노드가 없는지 확인.
- `pkill -f <패턴>`은 그 패턴이 들어간 자기 셸 명령까지 죽일 수 있으니 PID로 종료.
- 한 번에 여러 설정을 바꾸지 말고 기준선에서 하나씩.
- 허브가 부하에서 반복해서 무너져 카메라/Arduino가 빠졌다. 빠지면 `usb_cam` 노드도 죽으므로 launch를 다시 켤 것.

## 문서/도구 위치 (자세한 내용)

- `docs/lane_following_guide.md`: 현재 상태, 조향이 만들어지는 수식/단위, 파라미터 레퍼런스, 튜닝 순서, 앞으로 바꿀 것(점선 병행 설계 포함).
- `scripts/lane_tuning/`: `calib_pose.py`(기준자세 보정), `bag_summary.py`, `bag_param_sweep.py`, `bag_montage.py`(rosbag 기반 분석).

## USB 연결 구조 (사용자 확인)

노트북 USB-A → "A to A 허브" → ① Arduino ② 다른 허브(카메라 2대 + 라이다). 허브가 2단이라 `lsusb -t`에서 허브 안에 허브가 보인다.
