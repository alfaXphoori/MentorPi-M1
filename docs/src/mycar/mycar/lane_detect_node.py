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
        # Change to specialized topic for multiplexer
        self.publisher_ = self.create_publisher(Twist, '/lane_vel', 10)
        self.subscription = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        
        # Declare and Get Parameters
        self.declare_parameter('lower_yellow', [20, 100, 100])
        self.declare_parameter('upper_yellow', [40, 255, 255])
        self.declare_parameter('lower_white', [0, 0, 180]) # Default white range
        self.declare_parameter('upper_white', [180, 50, 255])
        self.declare_parameter('use_white', False) # Option to switch to white lines
        self.declare_parameter('base_speed', 0.15)
        self.declare_parameter('kp', 0.005)
        self.declare_parameter('show_debug', True)

        self.lower_yellow = np.array(self.get_parameter('lower_yellow').value, dtype=np.uint8)
        self.upper_yellow = np.array(self.get_parameter('upper_yellow').value, dtype=np.uint8)
        self.lower_white = np.array(self.get_parameter('lower_white').value, dtype=np.uint8)
        self.upper_white = np.array(self.get_parameter('upper_white').value, dtype=np.uint8)
        self.use_white = self.get_parameter('use_white').value
        self.base_speed = self.get_parameter('base_speed').value
        self.kp = self.get_parameter('kp').value
        self.show_debug = self.get_parameter('show_debug').value

        self.get_logger().info(f'Lane Detection Started. Mode: {"White" if self.use_white else "Yellow"}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'CV Bridge Error: {e}')
            return

        try:
            h, w, _ = cv_image.shape
            
            # 1. ROI 
            roi_top = int(h * 0.65)
            roi_bottom = int(h * 0.95)
            roi = cv_image[roi_top:roi_bottom, 0:w]
            
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            # 2. Filtering
            if self.use_white:
                mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
            else:
                mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            
            # 3. Noise reduction
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # 4. Find line center
            M = cv2.moments(mask)
            twist = Twist()
            
            if M['m00'] > 500: 
                cx = int(M['m10']/M['m00'])
                error = cx - w/2
                
                twist.linear.x = self.base_speed
                twist.angular.z = -float(error) * self.kp
                
                if self.show_debug:
                    cv2.circle(roi, (cx, int((roi_bottom-roi_top)/2)), 10, (0, 0, 255), -1)
                    cv2.line(roi, (int(w/2), 0), (int(w/2), roi_bottom-roi_top), (255, 0, 0), 2)
            else:
                # Line lost logic: stop or slow turn to find it
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                
            self.publisher_.publish(twist)

            # Show results only if enabled
            if self.show_debug:
                cv2.imshow("Lane Mask", mask)
                cv2.imshow("Lane ROI", roi)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Processing Error: {e}')


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
