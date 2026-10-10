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
