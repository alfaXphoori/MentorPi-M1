#!/usr/bin/env python3
# encoding: utf-8
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import os

class LaneDetectNode(Node):
    def __init__(self):
        super().__init__('lane_detect_node')
        
        # Publish directly to robot base controller
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriptions
        self.subscription = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',
            self.image_callback,
            10)
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
            
        self.bridge = CvBridge()
        self.debug_pub = self.create_publisher(Image, '/lane_debug', 10)
        
        # State variables
        self.current_yaw = 0.0
        self.imu_received = False
        self.last_error = 0.0
        self.searching = False
        self.target_yaw = 0.0
        self.weight_sum = 1.0
        
        # Declare and Get Parameters
        self.declare_parameter('lower_yellow', [20, 100, 100])
        self.declare_parameter('upper_yellow', [40, 255, 255])
        self.declare_parameter('lower_white', [0, 0, 180])
        self.declare_parameter('upper_white', [180, 50, 255])
        self.declare_parameter('use_white', False)
        self.declare_parameter('base_speed', 0.15)
        self.declare_parameter('kp', 0.005)
        self.declare_parameter('show_debug', True)
        
        # ROI parameters (y_start, y_end, x_start, x_end, weight) - Ratios
        # These will be scaled to image size in callback
        self.roi_configs = [
            (0.85, 0.95, 0.0, 1.0, 0.7),
            (0.70, 0.85, 0.0, 1.0, 0.2),
            (0.55, 0.70, 0.0, 1.0, 0.1)
        ]

        self.get_params()
        self.get_logger().info(f'Lane Detection Node Started. Mode: {"White" if self.use_white else "Yellow"}')

    def get_params(self):
        self.lower_yellow = np.array(self.get_parameter('lower_yellow').value, dtype=np.uint8)
        self.upper_yellow = np.array(self.get_parameter('upper_yellow').value, dtype=np.uint8)
        self.lower_white = np.array(self.get_parameter('lower_white').value, dtype=np.uint8)
        self.upper_white = np.array(self.get_parameter('upper_white').value, dtype=np.uint8)
        self.use_white = self.get_parameter('use_white').value
        self.base_speed = self.get_parameter('base_speed').value
        self.kp = self.get_parameter('kp').value
        self.show_debug = self.get_parameter('show_debug').value

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.imu_received = True

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def get_area_max_contour(self, contours, threshold=100):
        contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
        contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
        if len(contour_area) > 0:
            max_c_a = max(contour_area, key=lambda c_a: c_a[1])
            return max_c_a
        return None

    def process_lane(self, image, debug_image):
        h, w = image.shape[:2]
        centroid_sum = 0
        weights_used = 0
        center_x_list = []
        
        for r_config in self.roi_configs:
            y_start = int(h * r_config[0])
            y_end = int(h * r_config[1])
            x_start = int(w * r_config[2])
            x_end = int(w * r_config[3])
            weight = r_config[4]
            
            blob = image[y_start:y_end, x_start:x_end]
            contours = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]
            max_contour_area = self.get_area_max_contour(contours, 30)
            
            if max_contour_area is not None:
                rect = cv2.minAreaRect(max_contour_area[0])
                box = np.intp(cv2.boxPoints(rect))
                
                # Shift box to original image coordinates
                for j in range(4):
                    box[j, 1] += y_start
                    box[j, 0] += x_start
                
                if self.show_debug:
                    cv2.drawContours(debug_image, [box], -1, (255, 255, 0), 2)
                
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                line_center_x = (pt1_x + pt3_x) / 2
                line_center_y = (pt1_y + pt3_y) / 2
                
                if self.show_debug:
                    cv2.circle(debug_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)
                
                center_x_list.append((line_center_x, weight))
                centroid_sum += line_center_x * weight
                weights_used += weight
        
        if weights_used > 0:
            center_pos = centroid_sum / weights_used
            return center_pos
        return None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'CV Bridge Error: {e}')
            return

        self.get_params() # Update params in case they changed
        
        try:
            h, w, _ = cv_image.shape
            debug_image = cv_image.copy()
            
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # 2. Filtering
            if self.use_white:
                mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
            else:
                mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            
            # 3. Noise reduction
            kernel = np.ones((3,3), np.uint8)
            mask = cv2.erode(mask, kernel)
            mask = cv2.dilate(mask, kernel)
            
            # Process weighted ROIs
            center_pos = self.process_lane(mask, debug_image)
            
            twist = Twist()
            
            if center_pos is not None:
                self.searching = False
                error = center_pos - (w / 2.0)
                self.last_error = error
                
                # Angle calculation similar to provided snippet logic
                # angle = math.degrees(-math.atan(error / (h / 2.0)))
                # But we'll stick to PID/Kp control for ROS2 Twist
                
                twist.linear.x = self.base_speed
                twist.angular.z = -float(error) * self.kp
                
                if self.show_debug:
                    cv2.line(debug_image, (int(center_pos), 0), (int(center_pos), h), (0, 255, 0), 2)
                    cv2.putText(debug_image, f"Err: {error:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            else:
                # Line lost logic using IMU
                if not self.searching and self.imu_received:
                    self.searching = True
                    turn_direction = -1.0 if self.last_error > 0 else 1.0
                    self.target_yaw = self.normalize_angle(self.current_yaw + (turn_direction * math.pi / 2))
                    self.get_logger().info('Line lost! Searching using IMU...')

                if self.searching and self.imu_received:
                    yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
                    if abs(yaw_error) < 0.1:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                    else:
                        twist.linear.x = 0.05
                        twist.angular.z = 0.6 * yaw_error
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                
            self.publisher_.publish(twist)

            if self.show_debug:
                try:
                    debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                    self.debug_pub.publish(debug_msg)
                except Exception as e:
                    self.get_logger().error(f'Debug publish error: {e}')
                    
        except Exception as e:
            self.get_logger().error(f'Processing Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
