import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        
        # --- Configuration ---
        # 1. Yellow Lane (HSV)
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([40, 255, 255])
        
        # 2. Command Colors (Mean HSV Targets)
        self.green_h_range = (40, 90)
        self.red_h_range = (0, 15)  # or > 165
        self.min_sat_val = 80

        # 3. State
        self.is_active = False # Start as Stopped
        
        self.get_logger().info('Mission Control Started. Green to START, Red to STOP.')

    def image_callback(self, msg):
        full_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Work on a 320x240 version for speed
        img = cv2.resize(full_image, (320, 240))
        h, w, _ = img.shape
        
        # --- Part A: Color Command Detection (Center ROI) ---
        roi_size = 40
        cx, cy = w // 2, h // 2
        r = roi_size // 2
        color_roi = img[cy-r:cy+r, cx-r:cx+r]
        hsv_color = cv2.cvtColor(color_roi, cv2.COLOR_BGR2HSV)
        mean_hsv = cv2.mean(hsv_color)[:3]
        h_val, s_val, v_val = mean_hsv

        if s_val > self.min_sat_val and v_val > self.min_sat_val:
            if self.green_h_range[0] < h_val < self.green_h_range[1]:
                if not self.is_active:
                    self.get_logger().info('GO! Green detected.')
                    self.is_active = True
            elif h_val < self.red_h_range[1] or h_val > 165:
                if self.is_active:
                    self.get_logger().info('STOP! Red detected.')
                    self.is_active = False

        # --- Part B: Lane Following (Bottom ROI) ---
        # Use a strip at the bottom
        lane_roi_top = int(h * 0.75)
        lane_roi = img[lane_roi_top:h, 0:w]
        hsv_lane = cv2.cvtColor(lane_roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_lane, self.lower_yellow, self.upper_yellow)
        
        # Clean mask
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        twist = Twist()
        if self.is_active:
            M = cv2.moments(mask)
            if M['m00'] > 500:
                line_cx = int(M['m10']/M['m00'])
                error = line_cx - w/2
                
                twist.linear.x = 0.12 # Speed
                twist.angular.z = -float(error) * 0.006 # Steering
                
                # Visual center on lane_roi
                cv2.circle(lane_roi, (line_cx, 15), 5, (0,0,255), -1)
            else:
                self.get_logger().warn('Active but Line Lost!')
        else:
            # Stopped state
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher_.publish(twist)

        # --- Part C: Visualization ---
        # Draw Color ROI box
        cv2.rectangle(img, (cx-r, cy-r), (cx+r, cy+r), (255, 255, 0), 2)
        
        # Status Label
        status = "RUNNING" if self.is_active else "IDLE"
        color = (0, 255, 0) if self.is_active else (0, 0, 255)
        cv2.putText(img, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        cv2.imshow("Mission Control", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
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
