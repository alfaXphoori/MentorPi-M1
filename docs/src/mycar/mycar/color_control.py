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

        # Target Mean Colors (HSV)
        # Instead of ranges, we check if the average color is close to these
        self.green_target = np.array([60, 180, 150]) # Typical green
        self.red_target = np.array([0, 180, 150])   # Typical red
        
        # Tolerance for mean color matching
        self.hue_tolerance = 20
        self.sat_min = 70
        self.val_min = 70

        # Small ROI for ultra-speed (e.g., 40x40 pixels)
        self.roi_size = 40

        self.get_logger().info('Ultra-Fast Mean Color Control Started.')

    def timer_callback(self):
        self.publisher_.publish(self.current_twist)

    def image_callback(self, msg):
        # 1. Convert to CV2 (No resize yet)
        full_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = full_image.shape
        
        # 2. DIRECT CROP (The fastest way - process only 1600 pixels)
        cx, cy = w // 2, h // 2
        r = self.roi_size // 2
        roi = full_image[cy-r:cy+r, cx-r:cx+r]
        
        # 3. CALCULATE MEAN (Ultra-fast)
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mean_hsv = cv2.mean(hsv_roi)[:3] # [H, S, V]
        
        h_val, s_val, v_val = mean_hsv
        
        # 4. FAST LOGIC
        # Check Green (Hue around 60)
        if 40 < h_val < 90 and s_val > self.sat_min and v_val > self.val_min:
            if not self.is_moving:
                self.get_logger().info(f'Detected GREEN (Mean H:{int(h_val)}) - START')
                self.is_moving = True
                self.current_twist.linear.x = 0.2
        
        # Check Red (Hue near 0 or 180)
        elif (h_val < 15 or h_val > 165) and s_val > self.sat_min and v_val > self.val_min:
            if self.is_moving or self.current_twist.linear.x != 0.0:
                self.get_logger().info(f'Detected RED (Mean H:{int(h_val)}) - STOP')
                self.is_moving = False
                self.current_twist.linear.x = 0.0

        # 5. Visual Feedback (Optional - disable for maximum speed)
        # Draw on a small preview only to save CPU
        # preview = cv2.resize(roi, (160, 160)) # Zoomed in ROI
        # color = (0, 255, 0) if self.is_moving else (0, 0, 255)
        # cv2.putText(preview, f"H:{int(h_val)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        # cv2.rectangle(preview, (0,0), (159,159), color, 4)
        # 
        # cv2.imshow("Ultra-Fast ROI", preview)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ColorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
