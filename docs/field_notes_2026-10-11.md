# Field Notes — 2026-10-11 (네이티브 Ubuntu 22.04 환경 구축)

> 먼저 [인수인계 문서](handoff_2026-10-11.md)를 읽으세요. 이 문서는 그 문서에서 링크되는 상세 자료입니다.

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
- 노트북 포트가 부족함(연결 구조 전체는 [인수인계 문서](handoff_2026-10-11.md)에만 적어 둠). 충전 중인 USB-C 포트는 데이터 호스트로 동작하지 않음(`/sys/class/typec/port*/data_role`로 확인 가능).
  C→A 변환 어댑터가 인식되지 않는 경우가 있었으니(`partner=0`), 안 되면 다른 어댑터를 쓸 것.

## 테스트용 임시 설정 (로컬에만 있고 커밋하지 않음)

`s1_stack/config/s1_stack.yaml`: `th_scale: 200`(기본 1000, 저속 시험용), 조이스틱 축 번호.
커밋 전에 이 파일은 반드시 제외하거나 원래대로 되돌릴 것.

## 차선 추종 튜닝 (요약)

상세 수식/파라미터/튜닝 순서/개선 설계는 `docs/lane_following_guide.md`, 인수인계 요약은 `docs/handoff_2026-10-11.md`를 본다.
여기에는 현장에서 얻은 교훈만 남긴다.

- 후보 설정은 rosbag에 저장한 영상에 `lane_detect`를 다시 돌려(`scripts/lane_tuning/bag_param_sweep.py`) 먼저 거른 뒤 현장에서 하나만 시험한다.
- 조건을 완화(`min_slope 0.1`, `right1 200`)하면 주차 박스 가로선을 차선으로 잡아 조향이 ±1000으로 진동했다. `roi_top 280`은 위치가 다르면 검출률이 2%까지 붕괴했다. `th_scale 600`은 이탈.
- 시험용 노드를 띄우면 PID로 확실히 종료할 것. 남은 시험 노드가 `/cmd_lane`에 같이 발행해 기록이 오염됐다. 기록 전 `ros2 node list | sort | uniq -c`로 중복 확인.
- `pkill -f <패턴>`은 그 패턴이 들어간 자기 셸 명령까지 죽일 수 있으니 PID로 종료.
- 한 번에 여러 설정을 바꾸지 말고 기준선에서 하나씩.
- 허브가 부하에서 반복해서 무너져 카메라/Arduino가 빠졌다. 빠지면 `usb_cam`도 죽으므로 launch를 다시 켤 것.

## 문서/도구 위치 (자세한 내용)

- `docs/lane_following_guide.md`: 현재 상태, 조향이 만들어지는 수식/단위, 파라미터 레퍼런스, 튜닝 순서, 앞으로 바꿀 것(점선 병행 설계 포함).
- `scripts/lane_tuning/`: `calib_pose.py`(기준자세 보정), `bag_summary.py`, `bag_param_sweep.py`, `bag_montage.py`(rosbag 기반 분석).
