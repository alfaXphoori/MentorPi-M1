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

        # Broadened Color Ranges (HSV) for better detection
        self.green_lower = np.array([35, 50, 50])
        self.green_upper = np.array([90, 255, 255])
        
        self.red_lower1 = np.array([0, 70, 70])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([160, 70, 70])
        self.red_upper2 = np.array([180, 255, 255])

        # ROI Parameters (Proportional to resized image)
        self.target_width = 320
        self.target_height = 240
        self.roi_size = 80 # Detection box size

        self.get_logger().info('Optimized Color Control Started. Low-res mode enabled.')

    def timer_callback(self):
        self.publisher_.publish(self.current_twist)

    def image_callback(self, msg):
        # 1. Convert and Resize immediately for SPEED
        full_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image = cv2.resize(full_image, (self.target_width, self.target_height))
        
        height, width, _ = cv_image.shape
        
        # 2. Define ROI (Center)
        x1 = int(width/2 - self.roi_size/2)
        y1 = int(height/2 - self.roi_size/2)
        x2 = x1 + self.roi_size
        y2 = y1 + self.roi_size
        
        # 3. Process only the ROI
        roi = cv_image[y1:y2, x1:x2]
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        green_mask = cv2.inRange(hsv_roi, self.green_lower, self.green_upper)
        red_mask1 = cv2.inRange(hsv_roi, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(hsv_roi, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        green_pixels = cv2.countNonZero(green_mask)
        red_pixels = cv2.countNonZero(red_mask)

        # 4. Logic with Lower Threshold (More sensitive)
        if green_pixels > 300: 
            if not self.is_moving:
                self.get_logger().info(f'Green: {green_pixels} - STARTING')
                self.is_moving = True
                self.current_twist.linear.x = 0.2
        
        if red_pixels > 300:
            if self.is_moving or self.current_twist.linear.x != 0.0:
                self.get_logger().info(f'Red: {red_pixels} - STOPPING')
                self.is_moving = False
                self.current_twist.linear.x = 0.0

        # 5. Fast Debug View
        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (255, 255, 0), 2)
        status = "MOVING" if self.is_moving else "STOPPED"
        color = (0, 255, 0) if self.is_moving else (0, 0, 255)
        cv2.putText(cv_image, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(cv_image, f"G:{green_pixels} R:{red_pixels}", (10, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        cv2.imshow("Fast Color ROI", cv_image)
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
