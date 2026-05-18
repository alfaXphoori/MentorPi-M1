import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorControlNode(Node):
    def __init__(self):
        super().__init__('color_control_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        
        # Movement State
        self.is_moving = False
        self.current_twist = Twist()

        # Timer to publish command continuously at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Define Color Ranges (HSV)
        # Green range
        self.green_lower = np.array([35, 100, 100])
        self.green_upper = np.array([85, 255, 255])
        
        # Red range (Red wraps around in HSV, so we check two ranges)
        self.red_lower1 = np.array([0, 100, 100])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([160, 100, 100])
        self.red_upper2 = np.array([180, 255, 255])

        self.get_logger().info('Color Control Node Started. Green: Start, Red: Stop.')

    def timer_callback(self):
        self.publisher_.publish(self.current_twist)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Create masks
        green_mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        red_mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Check for presence (pixels count)
        green_pixels = cv2.countNonZero(green_mask)
        red_pixels = cv2.countNonZero(red_mask)

        status_text = "Wait for Color..."
        
        # Logic: Green to Start, Red to Stop
        if green_pixels > 5000: # Adjust threshold based on distance/size
            if not self.is_moving:
                self.get_logger().info('Green Detected - START MOVING')
                self.is_moving = True
                self.current_twist.linear.x = 0.2
        
        if red_pixels > 5000:
            if self.is_moving or self.current_twist.linear.x != 0.0:
                self.get_logger().info('Red Detected - STOP')
                self.is_moving = False
                self.current_twist.linear.x = 0.0

        # Debug visualization
        status_text = "MOVING (Green)" if self.is_moving else "STOPPED (Red/Idle)"
        color = (0, 255, 0) if self.is_moving else (0, 0, 255)
        
        cv2.putText(cv_image, f"Status: {status_text}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        cv2.putText(cv_image, f"G: {green_pixels} | R: {red_pixels}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

        cv2.imshow("Color Control Feedback", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ColorControlNode()
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
