# Field Notes — 2026-10-11 (네이티브 Ubuntu 22.04 환경 구축과 실차 자동 주행 첫 시험)

> 먼저 [인수인계 문서](../summary/handoff_2026-10-11.md)를 읽으세요. 이 문서는 그 문서에서 링크되는 상세 자료입니다.

10일 노트(`field_notes_2026-10-10.md`)의 결론대로 카메라 노드는 네이티브 Linux에서 돌려야 해서, 개발 머신을 Ubuntu 22.04.5(네이티브)로 옮겨 환경을 구축하고 처음으로 실차 하드웨어로 자동 주행 체인을 돌렸다.

반영 상태 표기: **커밋** = 저장소에 올라감 / **로컬** = 이 머신의 파일에만 있고 커밋 안 함 / **시스템** = 이 머신의 OS 설정(새 머신에서는 다시 해야 함) / **미해결**

---

## 1. 오늘 한 일 총정리

### 1.1 고친 문제

| # | 문제 | 원인 | 조치 | 반영 |
|---|---|---|---|---|
| 1 | Arduino가 몇 초마다 USB에서 끊김 (`disabled by hub (EMI?)`) | **USB 자동 절전**(`usbcore.autosuspend`=2초)이 놀고 있는 장치를 재울 때 허브 포트가 흔들림 | GRUB에 `usbcore.autosuspend=-1` | **시스템** (방법은 아래 3절, `vm_setup.sh`에 `NATIVE=1` 단계 **커밋**) |
| 2 | `serial_sender`가 USB가 끊기는 순간 죽음 (`write failed: Errno 5`) | 예외 처리·재연결이 없었음 | 끊기면 정지 상태로 대기하고 1초마다 재연결, 재연결 직후 2초는 전송 안 함 | **커밋** (`serial_sender.py`) |
| 3 | 자동 주행에서 조향이 한쪽 최대(+850~1000)로 쏠림 | 기준값 `guide_slope=1.11`, `guide_x=550`이 예전 카메라 설치용. 지금 카메라에서는 이상적인 자세에서도 각도 항이 -0.29 rad를 만듦 | 기준값을 파라미터로 열고 `calib_pose.py`로 측정(0.524, 531) | **커밋** (코드·도구). 보정값은 설정 파일이 아니라 **실행 명령에만** 있음 |
| 4 | 전진이 찔끔찔끔 | 차선 검출이 프레임마다 깜빡이면 검출 못 한 프레임에서 `linear.x=0` | `lost_hold_s`(놓친 뒤 전진·마지막 조향 유지, 넘기면 정지+조향 중앙 복귀) | **커밋** |
| 5 | `-p guide_x:=531` 같은 정수 인자가 `InvalidParameterTypeException` | 파라미터가 실수 타입으로만 선언됨 | 동적 타입 허용 | **커밋** |
| 6 | 자동 주행 중 앞으로 안 감 / `STALE INPUT - STOPPING` | 신호 입력(`/light_stop`, `/cross_stop`)이 없어서(`sign_detect` 꺼짐 또는 `cam_front` 없음). 버그가 아니라 안전 동작 | 사용법 문서화 (아래 4절). 코드 변경 없음 | **커밋**(문서) |
| 7 | 카메라 영상이 ROS 토픽으로 안 들어옴 (0Hz) | `~/.bashrc`의 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. 같은 머신에서 FastDDS로 바꾸면 영상이 들어옴 (아래 5절) | `.bashrc`에서 해당 줄 주석 처리. 설치 스크립트가 `NATIVE=1`이면 이 줄을 넣지 않게 수정 | **로컬**(.bashrc) + **커밋**(`vm_setup.sh`) |
| 8 | `scripts/s1_status.sh`가 `Ctrl+C`도 안 먹고 안 끝남 | `ros2 topic hz`가 `timeout`의 종료 신호를 무시 | `timeout -s KILL` | **커밋** |
| 9 | README 오류 | 정지 버튼이 X로 적힘(실제 Y), 펌웨어 경로 오타(`Arduino/S1.ino`), 해결된 이슈가 미확인으로 남음 | 정정 | **커밋** |
| 10 | 한글 입력 불가 | ibus-hangul의 시작 모드가 영문(`latin`) | `gsettings set org.freedesktop.ibus.engine.hangul initial-input-mode 'hangul'` | **시스템**(개인 설정) |

### 1.2 만든 것

`lane_trace` 파라미터, `scripts/lane_tuning/`, 문서, `.gitignore`는 모두 **커밋**됐다. 파일별 변경 목록은 [인수인계 문서 7절](../summary/handoff_2026-10-11.md).

### 1.3 커밋하지 않은 것 (이 머신에만 있음)

저장소 안:

| 파일 | 내용 | 이유 |
|---|---|---|
| `s1_stack/config/s1_stack.yaml` | `th_scale: 450`(기본 1000), `axis_steer: 3`, `axis_drive: 1`(Xbox 컨트롤러 기준) | 시험용 속도 제한이고 조이스틱 축은 컨트롤러 의존적. 팀원 설정을 바꾸면 안 됨. 설정 방법은 인수인계 문서 2절 ⑤ |

이 머신의 OS/사용자 설정 (저장소 밖, 새 머신에서는 직접 해야 함):

| 설정 | 내용 |
|---|---|
| GRUB | `usbcore.autosuspend=-1` (`/etc/default/grub`). `NATIVE=1 bash scripts/vm_setup.sh`로 재현 가능 |
| `ModemManager` | 중지·비활성화함. **결국 원인이 아니었으므로 되돌려도 됨** (`sudo systemctl enable --now ModemManager`) |
| `~/.bashrc` | CycloneDDS 줄 주석 처리, `source ~/FMTC_intern/install/setup.bash` 추가 |
| udev 규칙 | `/etc/udev/rules.d/99-robot.rules` (설치 스크립트가 복사). 장치 시리얼로 `/dev/arduino_mega` 등 이름이 정해짐 |
| 그룹 | `dialout`, `video` (재로그인/재부팅 후 적용) |
| git | 이 저장소 한정 `user.name`/`user.email`(`GangHeeJo <ganghee1245@snu.ac.kr>`), `gh auth login` 후 `gh auth setup-git`(전역 credential helper) |
| 패키지 | ROS2 humble(apt), `ultralytics`/`torch`/`pyserial`(pip, `~/.local`), `setuptools==58.2.0`, `numpy<2.0` |
| 한글 입력 | 위 1.1의 #10 |

임시 폴더(`/tmp/...`)에 있던 rosbag 기록과 분석 스크립트는 재부팅하면 사라진다. 분석 스크립트는 `scripts/lane_tuning/`으로 옮겨 커밋했고, rosbag 자체는 용량이 커서 저장하지 않았다.

### 1.4 미해결·미검증

| 구분 | 내용 |
|---|---|
| 미해결 | **코너에서 차선을 놓침**(오른쪽 경계선 하나만 보는 구조, 코너 구간 검출 약 26%) |
| 미해결 | 조향/전진이 번갈아 멈칫함 (명령은 끊기지 않았음 → 조향 모터와 구동 모터 전력 간섭 의심, 미검증) |
| 미해결 | 허브(2단)가 부하에서 장치를 한꺼번에 떨어뜨림 (`-71`/`-110`, `unable to enumerate`). 케이블 교체·재연결·재부팅으로 한 번 풀린 적 있으나 재발함 |
| 미해결 | 속도 `th_scale 600`은 조향이 못 따라가 이탈 (450까지만 확인) |
| 미해결 | Bluetooth/Wi-Fi 불가 (커널 6.8이 칩 미지원) → 조이스틱 유선, 인터넷 휴대폰 테더링 |
| 미해결 | 조향 게인(`a_rad`, `b_rad`)이 아직 파라미터로 안 열림 |
| 미검증 | 신호등/횡단보도 정지 (`sign_detect`는 돌았으나 `True`가 한 번도 안 뜸), 라이다·`obs_evade`(라이다 뺌), 주차는 미구현 |
| 미검증 | `min_slope:=0.35`, `steer_filter_n:=5`의 실주행 효과 (기록 영상 재생에서는 개선) |
| 미검증 | `vm_setup.sh`의 `NATIVE=1` 변경을 새 머신에서 실행해 봄 (문법 검사만 함) |

### 1.5 교훈

핵심 두 가지(한 번에 하나만 바꾸기, 시험 노드 종료)는 [인수인계 문서 10절](../summary/handoff_2026-10-11.md). 여기에는 추가분만 둔다.

- 남은 시험 노드(`lane_ptest`)가 `/cmd_lane`에 같이 발행해 **rosbag 일부(bag6, bag7)가 오염**됐고 그 기록으로 낸 결론(`roi_top 280`이 좋다)이 틀렸다.
- `pkill -f <패턴>`은 그 패턴이 들어간 자기 셸 명령까지 죽일 수 있으니 PID로 종료.
- 허브가 빠지면 `usb_cam` 노드가 죽으므로 장치가 돌아온 뒤 launch를 다시 켤 것.

---

## 2. 환경 구축

- `scripts/vm_setup.sh`가 네이티브 Ubuntu 22.04에서도 동작함. ROS2 humble, `usb_cam`/`cv_bridge`/`joy`, pip 패키지, 빌드, udev 규칙 복사까지 오류 없이 끝남.
- 검증: 전체 패키지(`lane_trace`, `sign_detect`, `s1_stack`, `obs_evade`, `gap_follow`, `wall_follow`, `sllidar_ros2`, `usb_cam`) `ros2 pkg list` 등록 확인. `s1_stack.launch.py`는 하드웨어 없이도 기동됨.
- 설치 스크립트는 sudo 비밀번호가 필요하므로 **일반 터미널에서** 실행 (Claude Code의 `!` 명령은 tty가 없어 비밀번호를 못 받음).
- `dialout`/`video` 그룹은 재로그인(또는 재부팅)해야 적용됨.
- 새 머신에 git 사용자 정보가 없으면 `git config user.name/user.email`을 먼저 설정.

## 3. USB 자동 절전(autosuspend) — Arduino 끊김의 원인

증상: Arduino(`/dev/arduino_mega`)가 몇 초마다 USB에서 끊겼다 붙으며 `usb 3-3.4-port3: disabled by hub (EMI?), re-enabling...`이 반복됨. 이때 `serial_sender`는 첫 쓰기에서 `write failed: [Errno 5] Input/output error`로 죽음.

- 코드/ModemManager/케이블/배터리 쪽이 아니었음. 노드를 하나도 안 돌려도 끊겼고, 반대로 launch로 카메라 스트리밍과 포트 사용 중일 때는 안정적이었음 → 장치가 놀 때 절전으로 들어가는 것이 단서.
- 확인: `usbcore.autosuspend = 2`(2초), 카메라와 안쪽 허브가 `runtime_status=suspended`.

```bash
# 임시 (재부팅하면 사라짐)
sudo sh -c 'echo -1 > /sys/module/usbcore/parameters/autosuspend; for f in /sys/bus/usb/devices/*/power/control; do echo on > $f; done'
# 영구 (또는 NATIVE=1 bash scripts/vm_setup.sh 가 해 줌)
sudo sed -i 's/^GRUB_CMDLINE_LINUX_DEFAULT="\(.*\)"/GRUB_CMDLINE_LINUX_DEFAULT="\1 usbcore.autosuspend=-1"/' /etc/default/grub
sudo update-grub
```

- 효과: 같은 허브 경유 상태에서 40초 동안 Arduino 재연결 11번 → 0번. 모터 구동 중에도 안 끊김.
- 비슷한 증상이면 `journalctl -k`의 `disabled by hub (EMI?)`와 `/sys/bus/usb/devices/*/power/runtime_status`부터 확인.
- 다만 허브 자체의 불안정(1.4 참고)은 이것과 별개로 남아 있다.

## 4. `STALE INPUT - STOPPING`이 나오는 이유

`decision_auto`는 기본값(`require_signals:=true`)에서 입력 3개가 모두 신선해야 움직인다.

| 입력 | 허용 지연 | 발행 노드 |
|---|---|---|
| `/cmd_lane` | 0.5초 | `lane_trace` |
| `/light_stop` | 1.0초 | `sign_detect` |
| `/cross_stop` | 1.0초 | `sign_detect` |

하나라도 빠지면 `linear.x = 0`. `/light_stop`·`/cross_stop`은 `/cam_front/image_raw`가 들어와야 발행되므로 전방 카메라 또는 `sign_detect`가 없으면 계속 정지한다.
카메라 1대로 차선만 시험할 때: `ros2 run s1_stack decision_auto --ros-args -p require_signals:=false` (`s1_stack.launch.py`는 이 파라미터를 넘기지 않는다).
`ros2 topic pub --once`처럼 한 번만 보낸 입력은 0.5초 뒤 stale이 되므로 `-r 10`으로 계속 발행해야 한다.

## 5. CycloneDDS가 네이티브에서 카메라 영상을 막음

같은 머신, 같은 카메라(640×480, 약 0.9MB/프레임)로 비교: 기본 FastDDS에서는 `ros2 topic hz`로 영상이 들어오고 `lane_trace`가 약 12fps로 처리했다. `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`에서는 **0장**이었다. V4L2 수준(`v4l2-ctl`)은 30fps로 정상이었다.
원인은 큰 메시지가 소켓 버퍼 한계(`net.core.rmem_max=212992`)에 걸리는 것으로 **추정**(확정 아님). CycloneDDS는 WSL2/VM에서 노드끼리 서로 못 찾는 문제를 피하려고 넣은 것이라 네이티브에서는 필요 없다.

## 6. `serial_sender` 재연결 동작

끊기면 전송을 멈추고(정지 유지) 1초마다 재연결을 시도하며, 재연결 직후 Arduino 부팅 시간(2초)은 전송하지 않고, 이전 명령은 재전송하지 않는다(항상 정지에서 다시 시작). 포트를 열 때마다 Arduino가 리셋되므로 launch 직후 약 10초는 기다린 뒤 조작할 것.
(참고: `serial_sender`의 `timeout_s` 파라미터는 선언만 되어 있고 실제로는 쓰이지 않는다. 명령이 끊기면 Arduino 펌웨어가 2초 뒤 정지한다.)

## 7. 조이스틱 (Xbox Series 컨트롤러, USB 유선)

- 버튼: 0=A(자동), 1=B(수동), 2=X, 3=**Y(정지)**.
- 축: 왼쪽 스틱 상하=1, 좌우=0, 오른쪽 스틱 상하=4, 좌우=3. 기본 설정(`axis_drive: 3`)은 다른 컨트롤러 기준이라 이 컨트롤러에서는 로컬에서 `axis_steer: 3`(오른쪽 스틱 좌우), `axis_drive: 1`(왼쪽 스틱 상하)로 바꿔 사용.
- `cmd_mux`는 STOP으로 시작하며 버튼을 눌러야 모드가 바뀜. STOP에서는 스틱을 무시.

## 8. 이 노트북(Samsung 940XHA, Lunar Lake)의 제약

- **블루투스와 Wi-Fi가 동작하지 않음**: 내장 칩(BT `8087:0037`, Wi-Fi `8086:a840`)에 커널 `6.8.0-138` 드라이버가 붙지 않아 `hci0`/`wlan`이 없음. 인터넷은 휴대폰 USB 테더링, 조이스틱은 유선만 가능.
- 노트북 포트가 3개뿐이고 모두 사용 중(연결 구조는 [인수인계 문서](../summary/handoff_2026-10-11.md)에만 적어 둠). 충전 중인 USB-C 포트는 데이터 호스트로 동작하지 않음(`/sys/class/typec/port*/data_role`로 확인).
- C→A 변환 어댑터가 노트북에서 인식되지 않은 적이 있었음(`partner=0`).

## 9. 무시해도 되는 로그

처음 보면 에러처럼 보이지만 동작에는 영향이 없다.

| 출력 | 어디서 | 의미 |
|---|---|---|
| `Could not initialize NNPACK! Reason: Unsupported hardware` (프레임마다 반복) | `sign_detect` | PyTorch의 CPU 가속 라이브러리를 이 CPU에서 못 쓴다는 알림. 다른 계산 방식으로 자동 전환됨. 오히려 영상을 받아 추론 중이라는 신호 |
| `unknown control 'white_balance_temperature_auto'`, `'exposure_auto'`, `'focus_auto'` | `usb_cam` | 카메라가 지원하지 않는 설정 항목을 건드렸다는 알림 |
| `Unable to open camera calibration file [.../default_cam.yaml]` | `usb_cam` | 카메라 보정 파일이 없다는 알림 (보정 없이 동작) |
| `KeyboardInterrupt` 스택 트레이스, `rcl_shutdown already called` | 거의 모든 노드 | `Ctrl+C`로 끌 때 나오는 정상 종료 과정 |
| `sllidar_node: Error, unexpected error, code: 80008004` | `sllidar_node` | 라이다를 안 꽂았을 때 |
| `Lane lost - stopping throttle` | `lane_trace` | 이번 프레임에서 차선을 못 찾았다는 알림 (`lost_hold_s` 동안은 전진 유지) |
| `sllidar_ros2` 빌드 경고 (`onDecodingError` 등) | `colcon build` | 벤더 코드의 unused-parameter 경고 |
