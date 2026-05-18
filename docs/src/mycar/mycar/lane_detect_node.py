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
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        
        # Optimized HSV for the Yellow Track from Image
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([40, 255, 255])

        self.get_logger().info('Lane Detection Optimized for Yellow Track Started.')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = cv_image.shape
        
        # 1. Focus on a narrow strip at the bottom (ROI) 
        # Looking too far ahead causes issues on sharp turns seen in the map
        roi_top = int(h * 0.7)
        roi_bottom = int(h * 0.95)
        roi = cv_image[roi_top:roi_bottom, 0:w]
        
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # 2. Filter for Yellow
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        # 3. Clean up noise (Morphological operations)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # 4. Find center of the line
        M = cv2.moments(mask)
        if M['m00'] > 1000: # Ensure enough pixels are found
            cx = int(M['m10']/M['m00'])
            
            # Visual feedback
            cv2.circle(roi, (cx, int((roi_bottom-roi_top)/2)), 10, (0, 0, 255), -1)
            cv2.line(roi, (int(w/2), 0), (int(w/2), roi_bottom-roi_top), (255, 0, 0), 2)
            
            # 5. Control Logic (Proportional)
            error = cx - w/2
            twist = Twist()
            twist.linear.x = 0.15 # Baseline speed
            
            # Kp (Proportional Gain) - adjust based on how "nervous" the car is
            kp = 0.005 
            twist.angular.z = -float(error) * kp
            
            self.publisher_.publish(twist)
        else:
            self.get_logger().warn('No Yellow Line in ROI!')
            # Optional: slow down or spin to find line
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)

        # Show results
        cv2.imshow("Yellow Mask", mask)
        cv2.imshow("Lane Tracking (ROI)", roi)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectNode()
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
