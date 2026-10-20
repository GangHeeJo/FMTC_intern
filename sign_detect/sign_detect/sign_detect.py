#!/usr/bin/env python3
# sign_detect: 전방 카메라 영상에서 YOLOv8로 횡단보도/신호등을 찾아서
# /cross_stop, /light_stop (둘 다 Bool, True면 "정지해야 함")을 발행하는 노드.
# decision_auto가 이걸 받아서 차선 추종보다 우선으로 정지 명령을 내림.
# CPU 추론이라 느릴 수 있음(코어 많이 먹음) — 실행 시 taskset으로 전용 코어를 따로 배정해서 씀.

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
from collections import deque
from pathlib import Path
import cv2    # 1. 임포트 추가

class VisionPercept(Node):
    def __init__(self):
        super().__init__('sign_detect')

        # 비워두면 이 패키지와 함께 설치된 체크포인트를 사용
        self.declare_parameter('model_path', '')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path:
            model_path = Path(get_package_share_directory('sign_detect')) / 'best_0808.pt'
        model_path = Path(model_path).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f'YOLO model_path is not a file: {model_path}')

        self.declare_parameter('debug_view', False)
        # 빨간·노란불 확신도 기준. 10/19 정지선 앞(앞 카메라): 켜진 불 0.91~0.92, 꺼진 신호등을 빨강·노랑으로 잘못 봄 0.21~0.46
        self.declare_parameter('light_conf', 0.3)
        self.light_conf = float(self.get_parameter('light_conf').value)
        self.debug_view = self.get_parameter('debug_view').get_parameter_value().bool_value

        self.model = YOLO(str(model_path))
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)
        self.bridge = CvBridge()

        # 안정성 향상을 위한 윈도우 필터 (5프레임 저장)
        self.cw_history = deque(maxlen=5)
        self.tl_history = deque(maxlen=5)
        self.green_history = deque(maxlen=5)

        # 설정값
        self.img_width = 640  # 카메라 폭(픽셀). 아래 bbox_center_bound와 같이 "화면 가운데 근처인지" 판단에 씀
        # 신호등 상자 최소 넓이(px², 이 이상이어야 "충분히 가까워졌다"). 예전 보닛 카메라에서 신호등까지 거리를 대신하던 값.
        # 10/19 정지선 모드에서는 정지선이 거리를 맡으므로 낮춰도 됨(bag_full_193634: 96초 3924부터 빨강 0.88인데 15500 때문에 97.7초에야 인정)
        self.declare_parameter('min_bbox_size', 15500)
        self.min_bbox_size = int(self.get_parameter('min_bbox_size').value)
        self.bbox_center_bound = [0.1, 0.9]  # 횡단보도 박스 중심이 화면 가로 10~90% 안에 있어야 유효

        # Pub/Sub
        self.sub_img = self.create_subscription(
            Image, '/cam_front/image_raw', self.image_callback, qos_profile_sensor_data)
        
        self.pub_cross = self.create_publisher(Bool, 'cross_stop', 10)
        self.pub_light = self.create_publisher(Bool, 'light_stop', 10)
        # 초록불(10/19): decision_auto 정지선 모드는 빨간불이 잠깐 끊긴 것이 아니라 초록불을 봐야 출발
        self.pub_green = self.create_publisher(Bool, 'light_green', 10)

        self.get_logger().info('Sign Detect Node has been started.')

    def image_callback(self, msg):
        """카메라 프레임마다 호출: YOLO 추론 → 클래스별 판단 → 최근 5프레임 다수결로 최종 정지 여부 발행"""
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 추론 수행 (모델 클래스: 0=횡단보도, 1=초록불(무시), 2=빨간불, 3=노란불)
        results = self.model(cv_img, verbose=False)[0]
        
        # --- 디버그 창 (debug_view 파라미터로 on/off, 헤드리스 환경에서는 꺼야 함) ---
        if self.debug_view:
            debug_img = results.plot()
            cv2.imshow('YOLO Debug', debug_img)
            cv2.waitKey(1)
        # ------------------------
        
        current_cw_detected = False
        current_tl_detected = False
        current_green = False

        for box in results.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            coords = box.xyxy[0].tolist() 

            # 1. 횡단보도 판단 (Class 0)
            if cls == 0 and conf > 0.5:
                center_x = (coords[0] + coords[2]) / 2
                if self.bbox_center_bound[0] * self.img_width < center_x < self.bbox_center_bound[1] * self.img_width:
                    current_cw_detected = True

            # 초록불 (Green: 1). 크기 조건은 빨간불과 같음
            elif cls == 1 and conf > self.light_conf:
                if (coords[2] - coords[0]) * (coords[3] - coords[1]) >= self.min_bbox_size:
                    current_green = True

            # 2. 신호등 판단 (Red: 2, Yellow: 3)
            elif cls in [2, 3] and conf > self.light_conf:
                size = (coords[2] - coords[0]) * (coords[3] - coords[1])
                if size >= self.min_bbox_size:
                    current_tl_detected = True

        # 히스토리 업데이트 (최근 5프레임의 감지 여부만 기억)
        self.cw_history.append(current_cw_detected)
        self.tl_history.append(current_tl_detected)
        self.green_history.append(current_green)

        # 최종 결정: 최근 5프레임 중 3프레임 이상 감지돼야 정지로 확정 (한두 프레임 오검출 노이즈 제거)
        stop_cw = Bool()
        stop_cw.data = sum(self.cw_history) >= 3
        
        stop_tl = Bool()
        stop_tl.data = sum(self.tl_history) >= 3

        self.pub_cross.publish(stop_cw)
        self.pub_light.publish(stop_tl)
        self.pub_green.publish(Bool(data=sum(self.green_history) >= 3))

def main(args=None):
    rclpy.init(args=args)
    node = VisionPercept()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
