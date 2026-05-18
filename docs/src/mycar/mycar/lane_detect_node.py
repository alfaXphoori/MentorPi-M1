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
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        
        # Threshold for Yellow/White Line (Adjust as needed)
        # Using HSV range for bright yellow/white lines
        self.lower_line = np.array([20, 100, 100])
        self.upper_line = np.array([50, 255, 255])

        self.get_logger().info('Lane Detection Node Started.')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = cv_image.shape
        
        # 1. ROI: Look only at the bottom half of the screen
        roi = cv_image[int(h/2):h, 0:w]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # 2. Filter for line color
        mask = cv2.inRange(hsv, self.lower_line, self.upper_line)
        
        # 3. Find center of the line using Moments
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            # Visual feedback on ROI
            cv2.circle(roi, (cx, cy), 10, (0, 0, 255), -1)
            cv2.line(roi, (int(w/2), 0), (int(w/2), int(h/2)), (255, 0, 0), 2)
            
            # 4. Control Logic (Simple Proportional)
            error = cx - w/2
            twist = Twist()
            twist.linear.x = 0.1 # Move slowly
            twist.angular.z = -float(error) / 150.0 # Steer to correct error
            self.publisher_.publish(twist)
            
            self.get_logger().debug(f'Error: {error} | Steering: {twist.angular.z}')
        else:
            # If line is lost, stop or search
            self.get_logger().warn('Line Lost!')
            # self.publisher_.publish(Twist())

        # Debug windows
        cv2.imshow("Mask", mask)
        cv2.imshow("Lane Tracking", roi)
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
