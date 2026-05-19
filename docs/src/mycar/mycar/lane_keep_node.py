#!/usr/bin/env python3
# encoding: utf-8
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class LaneKeepNode(Node):
    def __init__(self):
        super().__init__('lane_keep_node')
        
        # Publisher for velocity commands (Sent to Mission Manager)
        self.publisher_ = self.create_publisher(Twist, '/lane_vel', 10)
        
        # Publisher for debug visualization
        self.debug_pub = self.create_publisher(Image, '/lane_keep_debug', 10)
        
        # Subscriber for camera feed
        self.subscription = self.create_subscription(
            Image, 
            '/ascamera/camera_publisher/rgb0/image', 
            self.image_callback, 
            10)
        self.bridge = CvBridge()
        
        # 3 ROIs Configuration (y_start, y_end, x_start, x_end, weight)
        # Optimized for curves: bottom ROI has highest weight
        self.rois = [
            (0.8, 0.9, 0.0, 1.0, 0.7), # Near (Bottom)
            (0.7, 0.8, 0.0, 1.0, 0.2), # Mid
            (0.6, 0.7, 0.0, 1.0, 0.1)  # Far (Top)
        ]
        
        # LAB Color Thresholds for Yellow
        self.lower_yellow = np.array([0, 0, 145])
        self.upper_yellow = np.array([255, 255, 255])
        
        # Control Parameters
        self.base_speed = 0.15
        self.kp = 0.008 # Proportional gain
        
        self.get_logger().info('Advanced Lane Keep Node (3 ROIs + LAB) Started.')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, _ = cv_image.shape
            
            # Pre-processing
            blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
            lab_img = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
            mask = cv2.inRange(lab_img, self.lower_yellow, self.upper_yellow)
            
            # Debug view initialization
            debug_img = cv_image.copy()
            
            centroid_sum = 0
            weight_sum = 0
            active_rois = 0
            
            for (y_s, y_e, x_s, x_e, weight) in self.rois:
                # Calculate pixel coordinates
                y1, y2 = int(h * y_s), int(h * y_e)
                x1, x2 = int(w * x_s), int(w * x_e)
                
                # Draw ROI boundary on debug
                cv2.rectangle(debug_img, (x1, y1), (x2, y2), (255, 0, 0), 1)
                
                # Crop mask for this ROI
                roi_mask = mask[y1:y2, x1:x2]
                M = cv2.moments(roi_mask)
                
                if M['m00'] > 200: # Threshold to ignore noise
                    cx = int(M['m10'] / M['m00']) + x1
                    cy = int(M['m01'] / M['m00']) + y1
                    
                    # Draw centroid
                    cv2.circle(debug_img, (cx, cy), 5, (0, 0, 255), -1)
                    
                    centroid_sum += cx * weight
                    weight_sum += weight
                    active_rois += 1
                
            # Final Decision
            twist = Twist()
            if active_rois > 0:
                target_x = centroid_sum / weight_sum
                error = target_x - w/2
                
                twist.linear.x = self.base_speed
                twist.angular.z = -float(error) * self.kp
                
                # Draw steering line
                cv2.line(debug_img, (w//2, h), (int(target_x), int(h*0.7)), (0, 255, 0), 3)
                cv2.putText(debug_img, f"Err: {int(error)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(debug_img, "LINE LOST", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            
            self.publisher_.publish(twist)
            
            # Publish Debug View
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Lane Keep Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LaneKeepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
