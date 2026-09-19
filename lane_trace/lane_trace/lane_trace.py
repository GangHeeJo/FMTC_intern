#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

def color_filter(image):
    # 햇빛 변화에 강한 HLS 색 공간 변환
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    
    # 흰색 차선 필터링 (조도에 따라 L값 범위를 조절하세요)
    lower_white = np.array([0, 100, 0])      #150~200
    upper_white = np.array([255, 255, 255])
    
    mask = cv2.inRange(hls, lower_white, upper_white)
    return mask

def image_crop(image, start_point=(100, 220), end_point=(640, 450)):
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.rectangle(mask, start_point, end_point, 255, thickness=cv2.FILLED)
    return cv2.bitwise_and(image, image, mask=mask)

def get_x_coordinate(y_value, slope, y_intercept):
    if slope == 0: return 320
    return int((y_value - y_intercept) / slope)

def auto_canny(image, sigma=0.33):
    # 가우시안 블러로 노이즈 제거
    image_blur = cv2.GaussianBlur(image, (5, 5), 0)
    v = np.median(image_blur)
    
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image_blur, lower, upper) # 블러 처리된 이미지를 사용
    return edged

def lane_detect(image):

    result_overlay = image.copy()
    mask = color_filter(image)

    # 기존 ROI 유지
    start_point = (100, 220)
    end_point = (640, 450)
    mask_crop = image_crop(mask, start_point, end_point)
    
    # color_filter -> auto canny
    edge_crop = auto_canny(mask_crop, sigma=0.33)


    lines = cv2.HoughLinesP(edge_crop, rho=1, theta=np.pi/180, threshold=20, minLineLength=65, maxLineGap=10) # 70 -> 65
    
    filtered_points = []
    
    # 관심 영역 상수 (기존 유지)
    m, n, k = 0.3, (620, 380), 700
    right1, right2 = 330, 640
    
    # 필터링용 사선 및 수직선 (빨간색)
    cross_x = n[0] - k
    cross_y = int(n[1] - k * m)
    
    cv2.line(result_overlay, n, (cross_x, cross_y), (0, 0, 255), 1)
    cv2.line(result_overlay, (right1, 0), (right1, 480), (0, 0, 255), 1)
    cv2.line(result_overlay, (right2, 0), (right2, 480), (0, 0, 255), 1)

    # 가이드 및 조향 상수
    guide_x, guide_slope = 550, 1.11

    # --- 조향 가중치 수정 (0.4 rad 직접 계산용) ---
    a_rad, b_rad = 0.5, 0.0027  # a: 0.486 -> 0.5  /  b: 0.0017 -> 0.0035
    # ------------------------------------------

    slope_avg, i, sum_slope = guide_slope, 0, 0

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else 999.0
            midpoint = ((x1 + x2) / 2, (y1 + y2) / 2)
            
            # ROI 필터링: m_2(우측 상단 대각선) 조건을 제거하여 영역을 확장함
            if ((midpoint[0] > right1) and (midpoint[0] < right2) and 
                (m * (midpoint[0] - n[0]) + (n[1] - midpoint[1]) > 0) and 
                abs(slope) > 0.4):
                filtered_points.append((x1, y1))
                filtered_points.append((x2, y2))
                cv2.line(result_overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
                i += 1
                sum_slope += slope

    # 4. 최종 계산
    x_value = guide_x
    
    # 검출 여부를 확인하기 위한 플래그
    detected = False
    
    if i > 0:
        slope_avg = abs(np.arctan(sum_slope / i))
        detected = True # 차선 검출됨

    if len(filtered_points) > 0:
        points_arr = np.array(filtered_points)
        slope_fit, y_intercept_fit = np.polyfit(points_arr[:, 0], points_arr[:, 1], deg=1)
        x_value = get_x_coordinate(300, slope_fit, y_intercept_fit)
        
        x1_f, x2_f = int(min(points_arr[:, 0])), int(max(points_arr[:, 0]))
        cv2.line(result_overlay, (x1_f, int(slope_fit * x1_f + y_intercept_fit)), 
                 (x2_f, int(slope_fit * x2_f + y_intercept_fit)), (255, 0, 0), 3)

    # 검출 실패 시 None 반환
    if not detected:
        return result_overlay, mask, None

    # 5. 조향값 직접 계산 (이미 rad 단위이므로 Scaling 불필요)
    steering_rad = a_rad * (slope_avg - guide_slope) + b_rad * (x_value - guide_x)
    
    # 바로 0.4 rad로 클램핑
    steering_rad = np.clip(steering_rad, -0.4, 0.4)

    return result_overlay, mask, steering_rad


class LaneMaskingNode(Node):
    def __init__(self):
        super().__init__('lane_masking_node')
        self.cv_bridge = CvBridge()

        self.declare_parameter('debug_view', False)
        self.debug_view = self.get_parameter('debug_view').get_parameter_value().bool_value

        # Subscriber & Publisher
        self.img_sub = self.create_subscription(Image, '/cam_lane/image_raw', self.image_callback, 10)
        self.lane_change_sub = self.create_subscription(Bool, '/lane_change_flag', self.lane_change_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_lane', 10)
        
        self.lane_change_flag = 0
        self.last_steer_rad = 0.0  # 이전 조향값 저장용 변수 추가

    def lane_change_callback(self, msg):
        self.lane_change_flag = msg.data

    def image_callback(self, msg):
        image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        if self.lane_change_flag is True:
            image = cv2.flip(image, 1) # 이미지 반전
            
        result_img, mask_img, steer_rad = lane_detect(image)
        
        # --- 수정된 로직 ---
        if steer_rad is not None:
            # 새로운 값이 들어왔을 때만 업데이트
            self.last_steer_rad = steer_rad
        else:
            # 검출 실패 시 이전 값 유지 (로그로 표시하면 디버깅에 좋음)
            self.get_logger().warn('Lane lost - maintaining last steering value')
        
        # 현재(또는 유지된) 조향값 사용
        current_steer = self.last_steer_rad
        # -----------------        
        
        # --- [추가] 조향 부호 반전 로직 ---
        if self.lane_change_flag is True:
            current_steer = -current_steer  # 이미지 반전에 맞춰 조향 방향도 반대로 뒤집음
        # -------------------------------
        
        steer_deg = np.degrees(current_steer)
        self.get_logger().info(f'Steering -> Rad: {current_steer:.3f}, Deg: {steer_deg:.1f}°')
        
        if self.debug_view:
            cv2.imshow("Lane Detection (Overlay)", result_img)
            cv2.imshow("White Mask (Binary)", mask_img)
            cv2.waitKey(1)
        
        # 제어 메시지 생성
        drive_msg = Twist()
        drive_msg.linear.x = 0.5  # 고정 속도
        # 0.4 rad일 때 1000이 되도록 맵핑 (1000 / 0.4 = 2500)
        drive_msg.angular.z = -float(current_steer * 2500.0)
        
        self.cmd_pub.publish(drive_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = LaneMaskingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
