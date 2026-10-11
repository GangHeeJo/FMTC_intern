#!/usr/bin/env bash
# 차가 안 움직일 때 원인을 빠르게 좁히기 위한 상태 스냅샷.
# 실행 전: source install/setup.bash 되어 있어야 함.
set -u
export PYTHONUNBUFFERED=1   # 파이프로 받을 때 출력이 버퍼에 갇히지 않게

echo "===== nodes ====="
timeout -s KILL 3 ros2 node list

echo
echo "===== drive_mode (0=MANUAL 1=AUTO 2=STOP) ====="
timeout -s KILL 2 ros2 topic echo /drive_mode --once 2>&1

echo
echo "===== topic rates (3초씩, 0이면 그 토픽이 문제) ====="
for t in /cmd_lane /light_stop /cross_stop /scan /joy /cmd_manual /cmd_auto /cmd_obs /cmd_out; do
  echo "--- $t ---"
  timeout -s KILL 3 ros2 topic hz "$t" 2>&1 | tail -3
done

echo
echo "===== serial_sender 상태(있으면) ====="
timeout -s KILL 2 ros2 topic echo /serial_ready --once 2>&1
timeout -s KILL 2 ros2 topic echo /serial_rx --once 2>&1

echo
echo "===== 마지막 /cmd_out 값 ====="
timeout -s KILL 2 ros2 topic echo /cmd_out --once 2>&1
