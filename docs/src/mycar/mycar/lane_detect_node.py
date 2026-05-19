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
        
        # State variables for IMU search and lane width
        self.current_yaw = 0.0
        self.imu_received = False
        self.last_error = 0.0
        self.searching = False
        self.target_yaw = 0.0
        self.last_lane_width = 300  # Default initial expected width
        
        # Declare and Get Parameters
        self.declare_parameter('lower_yellow', [20, 100, 100])
        self.declare_parameter('upper_yellow', [40, 255, 255])
        self.declare_parameter('lower_white', [0, 0, 180]) # Default white range
        self.declare_parameter('upper_white', [180, 50, 255])
        self.declare_parameter('use_white', False)
        self.declare_parameter('base_speed', 0.15)
        self.declare_parameter('kp', 0.005)
        self.declare_parameter('show_debug', True)
        self.declare_parameter('target_x_ratio', 0.5) # Default 0.5 = Center of lane
        self.declare_parameter('roi_top_ratio', 0.45) # Look further ahead (default was 0.65)
        self.declare_parameter('roi_bottom_ratio', 0.95)

        self.lower_yellow = np.array(self.get_parameter('lower_yellow').value, dtype=np.uint8)
        self.upper_yellow = np.array(self.get_parameter('upper_yellow').value, dtype=np.uint8)
        self.lower_white = np.array(self.get_parameter('lower_white').value, dtype=np.uint8)
        self.upper_white = np.array(self.get_parameter('upper_white').value, dtype=np.uint8)
        self.use_white = self.get_parameter('use_white').value
        self.base_speed = self.get_parameter('base_speed').value
        self.kp = self.get_parameter('kp').value
        self.show_debug = self.get_parameter('show_debug').value
        self.target_x_ratio = self.get_parameter('target_x_ratio').value
        self.roi_top_ratio = self.get_parameter('roi_top_ratio').value
        self.roi_bottom_ratio = self.get_parameter('roi_bottom_ratio').value

        self.get_logger().info(f'Dual-Lane Detection Started. Mode: {"White" if self.use_white else "Yellow"}')

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
            roi_top = int(h * self.roi_top_ratio)
            roi_bottom = int(h * self.roi_bottom_ratio)
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
            
            # 4. Split into Left and Right halves
            mid_x = w // 2
            mask_left = mask[:, :mid_x]
            mask_right = mask[:, mid_x:]

            M_left = cv2.moments(mask_left)
            M_right = cv2.moments(mask_right)

            left_found = M_left['m00'] > 200
            right_found = M_right['m00'] > 200

            cx_left = int(M_left['m10']/M_left['m00']) if left_found else None
            cx_right = int(M_right['m10']/M_right['m00']) + mid_x if right_found else None

            twist = Twist()
            error = None

            if left_found and right_found:
                # Both found: update lane width and calculate center
                self.last_lane_width = cx_right - cx_left
                center_lane = (cx_left + cx_right) / 2
                self.searching = False
                
            elif left_found and not right_found:
                # Only left found: estimate right line
                center_lane = cx_left + (self.last_lane_width / 2)
                self.searching = False
                
            elif right_found and not left_found:
                # Only right found: estimate left line
                center_lane = cx_right - (self.last_lane_width / 2)
                self.searching = False
                
            else:
                center_lane = None

            if center_lane is not None:
                if self.searching:
                    self.get_logger().info('Line found! Resuming dual-lane tracking.')
                
                target_x = int(w * self.target_x_ratio)
                error = center_lane - target_x
                self.last_error = error
                
                twist.linear.x = self.base_speed
                twist.angular.z = -float(error) * self.kp
                
                if self.show_debug:
                    y_pos = int((roi_bottom-roi_top)/2)
                    # Draw detected lines
                    if left_found:
                        cv2.circle(roi, (cx_left, y_pos), 10, (0, 0, 255), -1)
                        cv2.putText(roi, "L", (cx_left-10, y_pos-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    if right_found:
                        cv2.circle(roi, (cx_right, y_pos), 10, (255, 0, 0), -1)
                        cv2.putText(roi, "R", (cx_right-10, y_pos-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    
                    # Draw calculated lane center
                    cv2.circle(roi, (int(center_lane), y_pos), 8, (0, 255, 255), -1)
                    
                    # Draw our target tracking line (green line)
                    cv2.line(roi, (target_x, 0), (target_x, roi_bottom-roi_top), (0, 255, 0), 2)

            else:
                # Line lost logic: use IMU to turn and find it
                if not self.searching and self.imu_received:
                    self.searching = True
                    turn_direction = -1.0 if self.last_error > 0 else 1.0
                    self.target_yaw = self.normalize_angle(self.current_yaw + (turn_direction * math.pi / 2))
                    self.get_logger().info('Both lines lost! Searching using IMU...')

                if self.searching and self.imu_received:
                    yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
                    if abs(yaw_error) < 0.1: # Reached target
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                    else:
                        twist.linear.x = 0.08
                        twist.angular.z = 0.8 * yaw_error
                        
                        max_turn = 0.5
                        min_turn = 0.2
                        if twist.angular.z > 0:
                            twist.angular.z = max(min(twist.angular.z, max_turn), min_turn)
                        else:
                            twist.angular.z = min(max(twist.angular.z, -max_turn), -min_turn)
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                
            self.publisher_.publish(twist)

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
