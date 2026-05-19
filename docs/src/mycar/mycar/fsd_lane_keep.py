#!/usr/bin/env python3
# encoding: utf-8
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class FSDLaneKeep(Node):
    def __init__(self):
        super().__init__('fsd_lane_keep')
        
        # Publish to intermediate topic for Mission Manager
        self.publisher_ = self.create_publisher(Twist, '/fsd/lane_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/fsd/lane_debug', 10)
        
        self.subscription = self.create_subscription(
            Image, '/ascamera/camera_publisher/rgb0/image', self.image_callback, 10)
        self.bridge = CvBridge()
        
        # 3 ROIs for stable tracking (y_start, y_end, x_start, x_end, weight)
        self.rois = [
            (0.8, 0.95, 0.0, 1.0, 0.7), # Near
            (0.7, 0.8, 0.0, 1.0, 0.2),  # Mid
            (0.6, 0.7, 0.0, 1.0, 0.1)   # Far
        ]
        
        # LAB Yellow thresholds
        self.lower_yellow = np.array([0, 0, 145])
        self.upper_yellow = np.array([255, 255, 255])
        
        self.base_speed = 0.15
        self.kp = 0.008
        
        self.get_logger().info('FSD Lane Keep Node Started.')

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, _ = cv_img.shape
            lab = cv2.cvtColor(cv_img, cv2.COLOR_BGR2LAB)
            mask = cv2.inRange(lab, self.lower_yellow, self.upper_yellow)
            
            # Cleaning
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            overlay = cv_img.copy()
            centroid_sum, weight_sum, active = 0, 0, 0
            
            for (y_s, y_e, x_s, x_e, weight) in self.rois:
                y1, y2, x1, x2 = int(h*y_s), int(h*y_e), int(w*x_s), int(w*x_e)
                roi_mask = mask[y1:y2, x1:x2]
                contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                if contours:
                    c = max(contours, key=cv2.contourArea)
                    M = cv2.moments(c)
                    if M['m00'] > 100:
                        cx = int(M['m10']/M['m00']) + x1
                        cy = int(M['m01']/M['m00']) + y1
                        
                        bx, by, bw, bh = cv2.boundingRect(c)
                        cv2.rectangle(overlay, (bx+x1, by+y1), (bx+bw+x1, by+bh+y1), (0, 255, 255), 2)
                        cv2.circle(overlay, (cx, cy), 5, (0,0,255), -1)
                        
                        centroid_sum += cx * weight
                        weight_sum += weight
                        active += 1
            
            twist = Twist()
            if active > 0:
                target_x = centroid_sum / weight_sum
                error = target_x - w/2
                twist.linear.x = self.base_speed
                twist.angular.z = -float(error) * self.kp
                cv2.line(overlay, (w//2, h), (int(target_x), int(h*0.7)), (0, 255, 0), 3)
                cv2.putText(overlay, f"Error: {int(error)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            self.publisher_.publish(twist)
            combined = np.vstack((overlay, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(combined, "bgr8"))
            
        except Exception as e:
            self.get_logger().error(f'FSD Lane Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FSDLaneKeep()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
