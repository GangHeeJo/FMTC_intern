#!/usr/bin/env python3
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8  # lane_change_flag용 (ROS1 커스텀 msg 대신)

from cv_bridge import CvBridge


def auto_canny(image, sigma=0.33):
    image_blur = cv2.GaussianBlur(image, (3, 3), 0)
    v = np.median(image_blur)
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)
    return edged


def image_crop(image, start_point=(0, 100), end_point=(640, 450)):
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.rectangle(mask, start_point, end_point, 255, thickness=cv2.FILLED)
    return cv2.bitwise_and(image, image, mask=mask)


def get_x_coordinate(y_value, slope, y_intercept):
    return int((y_value - y_intercept) / slope)


def lane_detect(image):
    start_point = (100, 220)
    end_point = (640, 450)

    edge = auto_canny(image)
    edge_crop = image_crop(edge, start_point, end_point)

    lines = cv2.HoughLinesP(
        edge_crop, rho=1, theta=np.pi/180, threshold=20,
        minLineLength=70, maxLineGap=10
    )
    result_color = cv2.cvtColor(edge_crop.copy(), cv2.COLOR_GRAY2BGR)

    filtered_points = []

    # ROI 상수 (원본 그대로)
    m = 0.3
    n = (620, 380)
    k = 700
    cross_x = n[0] - k
    cross_y = round(n[1] - k * m)

    m_2 = 2.5
    n_2 = (630, 400)
    cross_x_2 = n_2[0] - k
    cross_y_2 = round(n_2[1] - k * m_2)

    right1 = 330
    right2 = 640

    # 조향 상수 (원본 그대로)
    guide_x = 550
    guide_slope = 1.11
    a = 17
    b = 0.06

    slope_fit = guide_slope
    y_intercept_fit = 0.0
    slope_avg = guide_slope
    i = 0
    sum_slope = 0.0

    y_value = 300

    if lines is not None:
        # 가이드 라인 (디버그)
        cv2.line(result_color, n, (cross_x, cross_y), (0, 0, 255), 1)
        cv2.line(result_color, n_2, (cross_x_2, cross_y_2), (0, 0, 255), 1)
        cv2.line(result_color, (right1, 0), (right1, 480), (0, 0, 255), 1)
        cv2.line(result_color, (right2, 0), (right2, 480), (0, 0, 255), 1)

        for line in lines:
            x1, y1, x2, y2 = line[0]
            if (x2 - x1) == 0:
                continue
            slope = (y2 - y1) / (x2 - x1)
            midpoint = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

            if ((midpoint[0] > right1) and (midpoint[0] < right2) and
                (m * (midpoint[0] - n[0]) + (n[1] - midpoint[1]) > 0) and
                (m_2 * (midpoint[0] - n_2[0]) + (n_2[1] - midpoint[1]) < 0) and
                (abs(slope) > 0.4)):

                filtered_points.append((x1, y1))
                filtered_points.append((x2, y2))
                cv2.line(result_color, (x1, y1), (x2, y2), (0, 255, 0), 2)

                i += 1
                sum_slope += slope

        if i == 0:
            slope_avg = guide_slope
        elif i > 6:
            slope_avg = abs(np.arctan(sum_slope / i))
        else:
            slope_avg = np.arctan(sum_slope / i)

    # fitting
    if len(filtered_points) > 0:
        points_arr = np.array(filtered_points)
        fitted_line = np.polyfit(points_arr[:, 0], points_arr[:, 1], deg=1)
        slope_fit, y_intercept_fit = fitted_line

        x1_fit = int(min(points_arr[:, 0]))
        y1_fit = int(slope_fit * x1_fit + y_intercept_fit)
        x2_fit = int(max(points_arr[:, 0]))
        y2_fit = int(slope_fit * x2_fit + y_intercept_fit)

        cv2.line(result_color, (x1_fit, y1_fit), (x2_fit, y2_fit), (255, 0, 0), 2)
        cv2.line(result_color, (0, y_value), (640, y_value), (0, 230, 200), 1)

    # x절편 (기본값 보호)
    try:
        x_value = get_x_coordinate(y_value, slope_fit, y_intercept_fit)
    except Exception:
        x_value = guide_x

    steering_a = a * (guide_slope - slope_avg)
    steering_a = max(-14, min(14, steering_a))

    steering_b = b * (guide_x - x_value)
    steering_b = max(-14, min(14, steering_b))

    steering = steering_a + steering_b
    steering = max(-14, min(14, steering))

    # 횡단보도 같은 과검출 대응
    if i > 5 and abs(steering) > 5:
        steering = 0

    return result_color, float(steering), i


class CamLaneAuto(Node):
    def __init__(self):
        super().__init__('cam_lane_auto')
        self.bridge = CvBridge()

        # ---- params ----
        self.declare_parameter('image_topic', '/cam_lane/image_raw')
        self.declare_parameter('lane_change_topic', '/lane_change_flag')  # Int8: 1이면 좌우 반전
        self.declare_parameter('show_debug', True)

        self.declare_parameter('auto_speed', 0.7)          # Twist.linear.x (0~1로 쓰자)
        self.declare_parameter('steer_max_deg', 14.0)      # 기존 steering 범위
        self.declare_parameter('ang_max', 1.0)             # Twist.angular.z 최대값
        self.declare_parameter('min_lines_to_move', 1)     # 차선 검출 너무 약하면 정지(안전)

        self.image_topic = self.get_parameter('image_topic').value
        self.lane_change_topic = self.get_parameter('lane_change_topic').value
        self.show_debug = bool(self.get_parameter('show_debug').value)

        self.auto_speed = float(self.get_parameter('auto_speed').value)
        self.steer_max_deg = float(self.get_parameter('steer_max_deg').value)
        self.ang_max = float(self.get_parameter('ang_max').value)
        self.min_lines_to_move = int(self.get_parameter('min_lines_to_move').value)

        self.lane_change_flag = 0

        # ---- pubs/subs ----
        self.sub_img = self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            qos_profile_sensor_data
        )

        self.sub_flag = self.create_subscription(
            Int8,
            self.lane_change_topic,
            self.on_lane_change,
            10
        )

        self.pub_cmd_auto = self.create_publisher(Twist, '/cmd_auto', 10)

        self.get_logger().info(f"Subscribed image: {self.image_topic}")
        self.get_logger().info(f"Subscribed lane_change_flag(Int8): {self.lane_change_topic}")
        self.get_logger().info("Publishing /cmd_auto (Twist)")

    def on_lane_change(self, msg: Int8):
        self.lane_change_flag = int(msg.data)

    def on_image(self, msg: Image):
        
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge decode failed: {e}")
            return

        if self.lane_change_flag == 1:
            image = image[:, ::-1, :]  # 좌우 반전

        result, steering_deg, num_lines = lane_detect(image)

        if self.show_debug:
            cv2.imshow("cam_lane_result", result)
            cv2.waitKey(1)

        # steering_deg(-14..+14) -> angular.z(-ang_max..+ang_max)
        ang = (steering_deg / self.steer_max_deg) * self.ang_max
        ang = max(-self.ang_max, min(self.ang_max, ang))

        cmd = Twist()

        # 차선이 거의 안 잡히면 멈추는 안전 옵션
        if num_lines < self.min_lines_to_move:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = max(-1.0, min(1.0, self.auto_speed))
            cmd.angular.z = ang

        self.pub_cmd_auto.publish(cmd)


def main():
    rclpy.init()
    node = CamLaneAuto()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
