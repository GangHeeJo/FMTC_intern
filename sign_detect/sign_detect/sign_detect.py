#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import torch
from collections import deque
from pathlib import Path
import cv2
import math
import time

class VisionPercept(Node):
    def __init__(self):
        super().__init__('sign_detect')
        
        # Empty override uses the checkpoint installed with this ROS package.
        self.declare_parameter('model_path', '')
        self.declare_parameter('debug_view', False)
        self.debug_view = bool(self.get_parameter('debug_view').value)
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path:
            model_path = Path(get_package_share_directory('sign_detect')) / 'best_0808.pt'
        model_path = Path(model_path).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f'YOLO model_path is not a file: {model_path}')

        self.model = YOLO(str(model_path))
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)
        self.bridge = CvBridge()

        # 안정성 향상을 위한 윈도우 필터 (5프레임 저장)
        self.cw_history = deque(maxlen=5)
        self.tl_history = deque(maxlen=5)
        self.green_history = deque(maxlen=5)
        self.light_stop_latched = False
        # A new timestamp is necessary for an independent temporal vote; a
        # repeated ROS delivery must not turn one green image into three votes.
        self.image_timeout_s = 0.5
        self.last_image_stamp = None
        self.last_accepted_stamp = None
        self.last_accepted_wall = None

        # 설정값
        self.img_width = 640
        self.min_bbox_size = 15500
        self.bbox_center_bound = [0.1, 0.9]

        # Pub/Sub
        self.sub_img = self.create_subscription(
            Image, '/cam_front/image_raw', self.image_callback, qos_profile_sensor_data)
        
        self.pub_cross = self.create_publisher(Bool, 'cross_stop', 10)
        self.pub_light = self.create_publisher(Bool, 'light_stop', 10)

        self.get_logger().info('Sign Detect Node has been started.')

    def publish_stops(self, light_stop, cross_stop):
        stop_tl, stop_cw = Bool(), Bool()
        stop_tl.data, stop_cw.data = bool(light_stop), bool(cross_stop)
        self.pub_cross.publish(stop_cw)
        self.pub_light.publish(stop_tl)

    def reject_image(self):
        # Bool has no source timestamp. Never refresh a previous permissive
        # result on bad input; emit an explicit stop without changing votes.
        self.publish_stops(True, True)
        return False

    def image_callback(self, msg):
        """Return True only for a fresh committed result (used by wrappers).

        This gate validates source time, not traffic-head identity or image
        content uniqueness. Three independently stamped images can still show
        the wrong physical light; no intersection association is claimed here.
        """
        started = time.monotonic()
        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except (AttributeError, TypeError, ValueError):
            return self.reject_image()
        if (not math.isfinite(stamp) or not 0 <= now-stamp < self.image_timeout_s
                or (self.last_image_stamp is not None and stamp <= self.last_image_stamp)):
            return self.reject_image()
        # Reserve before inference as well: retrying a failed delivery is not
        # evidence from another captured frame.
        self.last_image_stamp = stamp

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_img, verbose=False)[0]
            if self.debug_view:
                debug_img = results.plot()
                cv2.imshow('YOLO Debug', debug_img)
                cv2.waitKey(1)

            current_cw_detected = False
            current_tl_detected = False
            current_green_detected = False
            for box in results.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                coords = box.xyxy[0].tolist()
                if len(coords) != 4 or not all(math.isfinite(v) for v in [conf, *coords]):
                    raise ValueError('nonfinite or malformed detector output')

                # Multiple boxes in one image contribute only one frame vote.
                if cls == 0 and conf > 0.5:
                    center_x = (coords[0] + coords[2]) / 2
                    if self.bbox_center_bound[0] * self.img_width < center_x < self.bbox_center_bound[1] * self.img_width:
                        current_cw_detected = True
                # Bundled checkpoint classes: Green=1, Red=2, Yellow=3.
                elif cls in [1, 2, 3] and conf > 0.3:
                    size = (coords[2] - coords[0]) * (coords[3] - coords[1])
                    if size >= self.min_bbox_size:
                        if cls in [2, 3]:
                            current_tl_detected = True
                        else:
                            current_green_detected = True
        except (CvBridgeError, cv2.error, ValueError, TypeError, IndexError, AttributeError, RuntimeError) as error:
            self.get_logger().warn('Signal frame rejected: ' + str(error))
            return self.reject_image()

        finished = time.monotonic()
        after = self.get_clock().now().nanoseconds * 1e-9
        if (not 0 <= after-stamp < self.image_timeout_s or after < now
                or not 0 <= finished-started < self.image_timeout_s):
            return self.reject_image()
        # Do not join evidence across a source gap or a frozen-clock outage.
        # A red latch survives; only three fresh consecutive greens release it.
        if self.last_accepted_stamp is not None and (
                stamp-self.last_accepted_stamp >= self.image_timeout_s
                or not 0 <= finished-self.last_accepted_wall < self.image_timeout_s):
            self.cw_history.clear()
            self.tl_history.clear()
            self.green_history.clear()
        self.last_accepted_stamp, self.last_accepted_wall = stamp, finished

        # 히스토리 업데이트
        self.cw_history.append(current_cw_detected)
        self.tl_history.append(current_tl_detected)

        # Red/yellow and crosswalk keep the existing three-of-five filtering.
        # Green release is stricter: three consecutive green-only fresh frames.
        # Missing detection or competing red/yellow breaks the green sequence.
        if current_tl_detected:
            self.green_history.clear()
            # Different visible heads cannot be associated with our lane here.
            # A simultaneous stop/go interpretation is immediately ambiguous.
            if current_green_detected or sum(self.tl_history) >= 3:
                self.light_stop_latched = True
        elif current_green_detected:
            self.green_history.append(True)
            if len(self.green_history) >= 3:
                self.light_stop_latched = False
                self.tl_history.clear()
        else:
            self.green_history.clear()

        self.publish_stops(self.light_stop_latched, sum(self.cw_history) >= 3)
        return True

def main(args=None):
    rclpy.init(args=args)
    node = VisionPercept()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.debug_view:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
