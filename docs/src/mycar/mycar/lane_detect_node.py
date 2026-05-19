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

class LaneDetectNode(Node):
    def __init__(self):
        super().__init__('lane_detect_node')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
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
        
        # Parameters for Fast Processing
        self.proc_w = 320
        self.proc_h = 240
        self.last_lane_width = 160.0 # Expected width in 320px image
        
        self.declare_parameter('use_white', True)
        self.declare_parameter('base_speed', 0.15)
        self.declare_parameter('kp', 0.015) # Adjusted for 320px width
        self.declare_parameter('show_debug', True)
        
        # Color Thresholds (Optimized for Indoor White Lines)
        self.lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
        self.upper_yellow = np.array([40, 255, 255], dtype=np.uint8)
        self.lower_white = np.array([0, 0, 200], dtype=np.uint8)
        self.upper_white = np.array([180, 60, 255], dtype=np.uint8)

        self.get_logger().info('FAST Lane Detection Started (320x240, Direct Moments).')

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

    def image_callback(self, msg):
        try:
            # 1. Decode & Resize for Speed
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            img = cv2.resize(cv_image, (self.proc_w, self.proc_h))
            
            # 2. Single ROI (Bottom 50%) - More context for better lookahead
            roi_top = int(self.proc_h * 0.5)
            roi = img[roi_top:self.proc_h, :]
            
            # Pre-processing: Blur to reduce reflection noise
            blur = cv2.GaussianBlur(roi, (5, 5), 0)
            
            # 3. HSV Masking
            hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
            if self.get_parameter('use_white').value:
                mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
            else:
                mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
                
            # Noise removal and bridging gaps
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.dilate(mask, kernel, iterations=1) # Bridge dashed lines
            
            # 4. Split L/R & Direct Moments (Skips findContours entirely -> Huge Speedup)
            mid_x = self.proc_w // 2
            mask_left = mask[:, :mid_x]
            mask_right = mask[:, mid_x:]
            
            M_l = cv2.moments(mask_left)
            M_r = cv2.moments(mask_right)
            
            # Require minimum area (e.g., 100 pixels) to consider valid
            cx_l = int(M_l['m10']/M_l['m00']) if M_l['m00'] > 100 else None
            cx_r = int(M_r['m10']/M_r['m00']) + mid_x if M_r['m00'] > 100 else None
            
            # 5. Logic
            center_pos = None
            if cx_l is not None and cx_r is not None:
                self.last_lane_width = cx_r - cx_l
                center_pos = (cx_l + cx_r) / 2
            elif cx_l is not None:
                center_pos = cx_l + (self.last_lane_width / 2)
            elif cx_r is not None:
                center_pos = cx_r - (self.last_lane_width / 2)
                
            twist = Twist()
            
            if center_pos is not None:
                if self.searching:
                    self.get_logger().info('Line found! Resuming.')
                self.searching = False
                
                error = center_pos - mid_x
                self.last_error = error
                
                base_s = self.get_parameter('base_speed').value
                kp = self.get_parameter('kp').value
                
                twist.linear.x = base_s
                twist.angular.z = -float(error) * kp
                
            else:
                # 6. Line lost: IMU turn fallback
                if not self.searching and self.imu_received:
                    self.searching = True
                    # Turn based on last known error direction
                    turn_direction = -1.0 if self.last_error > 0 else 1.0
                    self.target_yaw = self.normalize_angle(self.current_yaw + (turn_direction * math.pi / 2.5)) # ~72 deg turn
                    self.get_logger().info(f'Searching (Turn: {"Left" if turn_direction > 0 else "Right"})')

                if self.searching and self.imu_received:
                    yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
                    if abs(yaw_error) < 0.1:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.6 * yaw_error
                        # clamp turn speed to min/max to overcome static friction
                        max_turn = 0.8
                        min_turn = 0.35
                        if twist.angular.z > 0:
                            twist.angular.z = max(min_turn, min(twist.angular.z, max_turn))
                        else:
                            twist.angular.z = min(-min_turn, max(twist.angular.z, -max_turn))
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    
            self.publisher_.publish(twist)

            # 7. Fast Debug Visualization
            if self.get_parameter('show_debug').value:
                debug_img = img.copy()
                roi_center_y = roi_top + ((self.proc_h - roi_top) // 2)
                
                if cx_l is not None:
                    cv2.circle(debug_img, (cx_l, roi_center_y), 5, (0, 0, 255), -1)
                if cx_r is not None:
                    cv2.circle(debug_img, (cx_r, roi_center_y), 5, (255, 0, 0), -1)
                if center_pos is not None:
                    cv2.circle(debug_img, (int(center_pos), roi_center_y), 5, (0, 255, 255), -1)
                    cv2.line(debug_img, (int(center_pos), roi_center_y), (mid_x, self.proc_h), (0, 255, 0), 2)
                    
                cv2.putText(debug_img, f"Err: {self.last_error:.1f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
                self.debug_pub.publish(debug_msg)
                
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
