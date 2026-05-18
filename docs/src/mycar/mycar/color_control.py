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
        self.green_lower = np.array([35, 100, 100])
        self.green_upper = np.array([85, 255, 255])
        self.red_lower1 = np.array([0, 100, 100])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([160, 100, 100])
        self.red_upper2 = np.array([180, 255, 255])

        # ROI Parameters (Center Rectangle)
        self.roi_size = 200 # Square size of 200x200 pixels

        self.get_logger().info('Color Control with ROI Started. Show colors inside the rectangle.')

    def timer_callback(self):
        self.publisher_.publish(self.current_twist)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        height, width, _ = cv_image.shape
        
        # Define ROI coordinates (Center of screen)
        x1 = int(width/2 - self.roi_size/2)
        y1 = int(height/2 - self.roi_size/2)
        x2 = x1 + self.roi_size
        y2 = y1 + self.roi_size
        
        # Extract ROI
        roi = cv_image[y1:y2, x1:x2]
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Create masks for ROI only
        green_mask = cv2.inRange(hsv_roi, self.green_lower, self.green_upper)
        red_mask1 = cv2.inRange(hsv_roi, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(hsv_roi, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Count pixels in ROI
        green_pixels = cv2.countNonZero(green_mask)
        red_pixels = cv2.countNonZero(red_mask)

        # Logic
        if green_pixels > 2000: # Threshold is smaller because ROI is smaller
            if not self.is_moving:
                self.get_logger().info('Green in ROI - START MOVING')
                self.is_moving = True
                self.current_twist.linear.x = 0.2
        
        if red_pixels > 2000:
            if self.is_moving or self.current_twist.linear.x != 0.0:
                self.get_logger().info('Red in ROI - STOP')
                self.is_moving = False
                self.current_twist.linear.x = 0.0

        # Draw ROI Rectangle on main image
        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (255, 255, 0), 2)
        cv2.putText(cv_image, "Detection Area", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # Status visualization
        status_text = "MOVING" if self.is_moving else "STOPPED"
        color = (0, 255, 0) if self.is_moving else (0, 0, 255)
        cv2.putText(cv_image, f"Status: {status_text}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        cv2.imshow("Color ROI Feedback", cv_image)
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
