#!/usr/bin/env python3
# encoding: utf-8
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetectNode(Node):
    def __init__(self):
        super().__init__('lane_detect_node')
        
        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher for debug image (to view in RViz or Web)
        self.debug_pub = self.create_publisher(Image, '/lane_debug', 10)
        
        # Subscriber for camera feed
        self.subscription = self.create_subscription(
            Image, 
            '/ascamera/camera_publisher/rgb0/image', 
            self.image_callback, 
            10)
        self.bridge = CvBridge()
        
        # Yellow Line HSV Thresholds
        # These values are tuned to detect yellow lines clearly
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([45, 255, 255])
        
        # Parameters for Control
        self.base_speed = 0.1
        self.kp = 0.01 # Proportional gain
        
        self.get_logger().info('Yellow Lane Detection Node Started (Headless Mode).')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV BGR
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, _ = cv_image.shape
            
            # 1. Focus on the bottom half (Region of Interest)
            roi_h_start = int(h * 0.6)
            roi = cv_image[roi_h_start:h, 0:w]
            
            # 2. Pre-processing: Blur and convert to HSV
            blurred = cv2.GaussianBlur(roi, (5, 5), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            
            # 3. Detect Yellow Color
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            
            # Optional: Morphological operations to clean up the mask
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)
            
            # 4. Find the center of the detected line (Moments)
            M = cv2.moments(mask)
            
            # Create a debug image by combining ROI and the mask
            # We convert the 1-channel mask to 3-channels to overlay/concatenate
            mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # Draw visual markers on the original ROI
                cv2.circle(roi, (cx, cy), 7, (0, 0, 255), -1) # Red dot at centroid
                cv2.line(roi, (w//2, 0), (w//2, roi.shape[0]), (255, 0, 0), 1) # Blue centerline
                cv2.line(roi, (w//2, cy), (cx, cy), (0, 255, 0), 2) # Green error line
                
                # 5. Steering Logic
                error = cx - w/2
                
                twist = Twist()
                twist.linear.x = self.base_speed
                twist.angular.z = -float(error) * self.kp
                
                self.publisher_.publish(twist)
                
                # Add status text
                cv2.putText(roi, f"Error: {error}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            else:
                cv2.putText(roi, "LINE LOST", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                # Stop or slow down if line is lost
                self.publisher_.publish(Twist())

            # Tiled Visualization: Left (Original ROI), Right (Detection Mask)
            combined_view = np.hstack((roi, mask_rgb))
            
            # Publish Debug Image to ROS topic (for RViz or Web Server)
            # This avoids using cv2.imshow which fails in headless environments
            debug_msg = self.bridge.cv2_to_imgmsg(combined_view, "bgr8")
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Lane Detection Error: {e}')

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
