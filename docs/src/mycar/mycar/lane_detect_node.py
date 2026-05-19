import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class LaneDetectNode(Node):
    def __init__(self):
        super().__init__('lane_detect_node')
        # Publish directly to robot base controller
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',
            self.image_callback,
            10)
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        self.bridge = CvBridge()
        self.debug_pub = self.create_publisher(Image, '/lane_debug', 10)
        
        # State variables for IMU search
        self.current_yaw = 0.0
        self.imu_received = False
        self.last_error = 0.0
        self.searching = False
        self.target_yaw = 0.0
        
        # Declare and Get Parameters
        self.declare_parameter('lower_yellow', [20, 100, 100])
        self.declare_parameter('upper_yellow', [40, 255, 255])
        self.declare_parameter('lower_white', [0, 0, 180]) # Default white range
        self.declare_parameter('upper_white', [180, 50, 255])
        self.declare_parameter('use_white', False) # Option to switch to white lines
        self.declare_parameter('base_speed', 0.15)
        self.declare_parameter('kp', 0.005)
        self.declare_parameter('show_debug', True)
        self.declare_parameter('target_x_ratio', 0.8) # 0.5=Center, 0.2=Keep line on Left, 0.8=Keep line on Right

        self.lower_yellow = np.array(self.get_parameter('lower_yellow').value, dtype=np.uint8)
        self.upper_yellow = np.array(self.get_parameter('upper_yellow').value, dtype=np.uint8)
        self.lower_white = np.array(self.get_parameter('lower_white').value, dtype=np.uint8)
        self.upper_white = np.array(self.get_parameter('upper_white').value, dtype=np.uint8)
        self.use_white = self.get_parameter('use_white').value
        self.base_speed = self.get_parameter('base_speed').value
        self.kp = self.get_parameter('kp').value
        self.show_debug = self.get_parameter('show_debug').value
        self.target_x_ratio = self.get_parameter('target_x_ratio').value

        self.get_logger().info(f'Lane Detection Started. Mode: {"White" if self.use_white else "Yellow"}')

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.imu_received = True

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

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
                if self.searching:
                    self.get_logger().info('Line found! Resuming lane tracking.')
                self.searching = False
                cx = int(M['m10']/M['m00'])
                target_x = int(w * self.target_x_ratio)
                error = cx - target_x
                self.last_error = error
                
                twist.linear.x = self.base_speed
                twist.angular.z = -float(error) * self.kp
                
                if self.show_debug:
                    # Draw actual line center (red dot)
                    cv2.circle(roi, (cx, int((roi_bottom-roi_top)/2)), 10, (0, 0, 255), -1)
                    # Draw our target tracking line (green line)
                    cv2.line(roi, (target_x, 0), (target_x, roi_bottom-roi_top), (0, 255, 0), 2)
                    # Draw screen center for reference (thin blue line)
                    cv2.line(roi, (int(w/2), 0), (int(w/2), roi_bottom-roi_top), (255, 0, 0), 1)
            else:
                # Line lost logic: use IMU to turn and find it
                if not self.searching and self.imu_received:
                    self.searching = True
                    # Positive error -> line was on the right -> turn right (negative angular.z)
                    turn_direction = -1.0 if self.last_error > 0 else 1.0
                    # Set target yaw to 90 degrees from current
                    self.target_yaw = self.normalize_angle(self.current_yaw + (turn_direction * math.pi / 2))
                    self.get_logger().info('Curve detected! Lost line. Searching using IMU...')

                if self.searching and self.imu_received:
                    yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
                    if abs(yaw_error) < 0.1: # Reached target
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                    else:
                        twist.linear.x = 0.08 # Move forward slightly while turning
                        twist.angular.z = 0.8 * yaw_error
                        
                        # Clamp turn speed to prevent getting stuck or spinning too fast
                        max_turn = 0.5
                        min_turn = 0.2
                        if twist.angular.z > 0:
                            twist.angular.z = max(min(twist.angular.z, max_turn), min_turn)
                        else:
                            twist.angular.z = min(max(twist.angular.z, -max_turn), -min_turn)
                else:
                    # No IMU or not searching yet
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                
            self.publisher_.publish(twist)

            # Show results only if enabled
            if self.show_debug:
                try:
                    debug_msg = self.bridge.cv2_to_imgmsg(roi, "bgr8")
                    self.debug_pub.publish(debug_msg)
                except Exception as e:
                    self.get_logger().error(f'Debug publish error: {e}')
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
