# S1 차량 현장 확인용 안전 핫픽스

기존 S1 ROS2 주행 구조에 입력 단절·신호 판단·시리얼·Arduino 보정 처리를 보강한 실차 시험 후보입니다. **첫 주행 시험은 이 핫픽스본으로 진행합니다.** 새 lattice·경로 추종·후진 복구 코드는 이 저장소에 포함하지 않았습니다.

## 처음 실행할 때

1. 현장에서 사용하던 ROS2 노트북과 환경을 유지합니다. 차량별 핀·배선과 기존 펌웨어를 확인하고, 바퀴를 띄워 조향·구동·STOP을 확인합니다.
2. 센서 단계에서 LiDAR 방향·거리·끊김과 두 카메라 영상을 확인합니다.
3. 관측 단계에서 실제 차선과 신호 검출을 확인한 뒤 제한 출력 수동 시험, 저속 차선·신호 시험 순으로 진행합니다.
4. 시험한 커밋, 차량, 장치 경로, 적용한 펌웨어, 결과를 함께 기록합니다. 같은 차량의 다른 주행 스택은 종료한 뒤 전환합니다.

ROS2 작업공간의 `src` 아래에 이 저장소를 두고, 현장의 기존 의존성을 확인한 뒤 작업공간 루트에서 `colcon build --symlink-install`을 실행합니다. 각 터미널에서 해당 `install/setup.bash`를 읽습니다.

```bash
# Arduino 포트를 열지 않는 센서 확인
ros2 launch s1_stack commissioning.launch.py profile:=sensors

# 이전 launch 종료 후: 인식·판단만 관측, Arduino 미연결
ros2 launch s1_stack commissioning.launch.py profile:=observe

# 실제 연결·정지 수단 확인 후: Arduino 포트를 여는 제한 출력 수동 시험
ros2 launch s1_stack commissioning.launch.py profile:=manual max_throttle:=200

# 수동·센서 시험 통과 후: 저속 차선·신호 시험
ros2 launch s1_stack commissioning.launch.py profile:=auto max_throttle:=200
```

각 단계는 하나씩 실행합니다. `max_throttle=200`은 TH 명령 제한이며 속도 0.2m/s를 뜻하지 않습니다. 기본 `auto`에서는 기존 시간 기반 장애물 회피를 켜지 않습니다. 장치 경로·baud·카메라 설정·조이스틱 버튼과 Arduino 연결 시 자동 조향 보정 동작은 [현장 절차](docs/saturday_3h.md)를 먼저 확인합니다.

## 포함된 수정

- 차선 소실, 오래된 제어 입력, 조이스틱 단절에 대한 정지 처리.
- 빨간불 정지 유지, 서로 다른 새 초록 영상 3장의 연속 확인, 중복 영상·여러 검출 박스의 중복 집계 방지, 상충 신호·처리 중 만료된 영상의 정지 처리.
- Arduino 준비 회신과 새 중립 명령 확인, 시리얼 입력 유효시간 및 출력 제한.
- Arduino 보정 실패 시 구동 차단, 명령 파싱·보정 중 명령 처리 보강. 실제 차량에 자동 업로드하지 않습니다.

신호등·담당 차선 연결이나 정지선/교차로 진입 판정은 없습니다. 관련 한계와 변경 내용은 [공통 신호 핫픽스](docs/signal_hotfix_20261008.md)에 정리했습니다.

## 확인 도구와 검증 범위

- [환경 수집](scripts/s1_preflight.py): 환경·장치·소스를 읽고 보고서 저장. 구동 명령을 보내지 않습니다.
- [토픽 확인](scripts/s1_topic_check.py): 수신 상태와 발행자 수 기록. 구동 명령을 보내지 않습니다.
- [시험 결과 양식](docs/field_results_template.csv).

호스트 회귀시험:

```bash
python3 -m unittest discover -s tests -v
```

2026-10-08 안전 핫픽스 v2에서 76개 시험이 통과했습니다. 가짜 센서 입력·시계·시리얼과 호스트 펌웨어 시험 기준이며, ROS/DDS·현장 빌드·YOLO 정확도·실차 제동·차량 펌웨어 업로드 검증은 남아 있습니다. 수정 전 원본은 Git `b9a9dca`로 확인할 수 있습니다.
