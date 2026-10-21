# bag 분석 스크립트 (10/21)

실차 녹화(rosbag)를 텍스트·사진으로 요약하는 도구. 모두 `source /opt/ros/humble/setup.bash` 뒤에 실행.

| 스크립트 | 무엇을 보나 | 필요한 토픽 |
|---|---|---|
| `timeline.py` | 0.5초 단위 명령·신호·라이다 요약 (가장 먼저 볼 것) | `/cmd_*`, `/light_*`, `/scan`, `/lane_changing` … |
| `rosout_log.py` | `obs_evade`·`decision_auto` 로그 (단계 종료 이유, 복귀 판단, 신호 정지) | `/rosout` |
| `vision_replay.py` | vision 차선 변경 단계 조건을 `/lane_lines`로 재계산 | `/lane_lines`, `/cmd_obs` |
| `light_boxes.py` | 신호등 모델 결과(클래스·확신도·박스 넓이)를 시간별로 | `/cam_front/image_raw/compressed` |
| `lane_frames.py` | 차선 카메라 프레임 몽타주 | `/cam_lane/image_raw` |

## 녹화 명령

영상 없이(1분 수 MB):
```bash
cd ~/bags && ros2 bag record -o obs_$(date +%H%M%S) \
  /scan /rosout \
  /cmd_lane /cmd_obs /cmd_auto /cmd_out /drive_mode \
  /lane_lines /lane_detected /lane_slope /lane_changing /lane_change_flag /lane_mirror \
  /stop_line_y /crosswalk_active /light_stop /light_green
```
- 신호등 카메라를 볼 때: `/cam_front/image_raw/compressed` 추가(1초 약 1 MB).
- 차선 카메라를 볼 때: `/cam_lane/image_raw` 추가(1분 약 420 MB, FMTC 노트북 디스크가 작으니 1분 이내, 분석 뒤 삭제).

## 예
```bash
python3 scripts/bag_analysis/timeline.py ~/bags/obs_180128/*.db3
python3 scripts/bag_analysis/rosout_log.py ~/bags/obs_180128/*.db3
python3 scripts/bag_analysis/light_boxes.py ~/bags/obs_180551/*.db3 0.5 /tmp/front.jpg
python3 scripts/bag_analysis/lane_frames.py ~/bags/obs_175637/*.db3 /tmp/lane.jpg 11 13.4 16.5 18.6 19.5 21.4 22.5 23.3
python3 scripts/bag_analysis/vision_replay.py ~/bags/obs_175637/*.db3 10 26
```
