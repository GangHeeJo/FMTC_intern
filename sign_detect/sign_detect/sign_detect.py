#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
from collections import deque
import cv2    # 1. 임포트 추가

class VisionPercept(Node):
    def __init__(self):
        super().__init__('sign_detect')
        
        # 모델 설정 (경로가 올바른지 꼭 확인하세요!)
        self.declare_parameter('model_path', '/home/fmtc/s1_ws/src/sign_detect/best_0808.pt')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        
        self.model = YOLO(model_path)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)
        self.bridge = CvBridge()

        # 안정성 향상을 위한 윈도우 필터 (5프레임 저장)
        self.cw_history = deque(maxlen=5)
        self.tl_history = deque(maxlen=5)

        # 설정값
        self.img_width = 640
        self.min_bbox_size = 15500
        self.bbox_center_bound = [0.1, 0.9]

        # Pub/Sub
        self.sub_img = self.create_subscription(Image, '/cam_front/image_raw', self.image_callback, 10)
        
        self.pub_cross = self.create_publisher(Bool, 'cross_stop', 10)
        self.pub_light = self.create_publisher(Bool, 'light_stop', 10)

        self.get_logger().info('Sign Detect Node has been started.')

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 추론 수행
        results = self.model(cv_img, verbose=False)[0]
        
        # --- 디버그 창 추가 코드 ---
        debug_img = results.plot()           # 2. YOLO 박스 자동 그리기
        cv2.imshow('YOLO Debug', debug_img)  # 3. 창 띄우기
        cv2.waitKey(1)                      # 4. 화면 갱신 (필수)
        # ------------------------
        
        current_cw_detected = False
        current_tl_detected = False

        for box in results.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            coords = box.xyxy[0].tolist() 

            # 1. 횡단보도 판단 (Class 0)
            if cls == 0 and conf > 0.5:
                center_x = (coords[0] + coords[2]) / 2
                if self.bbox_center_bound[0] * self.img_width < center_x < self.bbox_center_bound[1] * self.img_width:
                    current_cw_detected = True

            # 2. 신호등 판단 (Red: 2, Yellow: 3)
            elif cls in [2, 3] and conf > 0.3:
                size = (coords[2] - coords[0]) * (coords[3] - coords[1])
                if size >= self.min_bbox_size:
                    current_tl_detected = True

        # 히스토리 업데이트
        self.cw_history.append(current_cw_detected)
        self.tl_history.append(current_tl_detected)

        # 최종 결정
        stop_cw = Bool()
        stop_cw.data = sum(self.cw_history) >= 3
        
        stop_tl = Bool()
        stop_tl.data = sum(self.tl_history) >= 3

        self.pub_cross.publish(stop_cw)
        self.pub_light.publish(stop_tl)

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
