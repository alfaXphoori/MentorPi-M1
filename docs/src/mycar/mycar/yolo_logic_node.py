import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces.msg import ObjectsInfo
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import cv2
import time
import math

class YoloLogicNode(Node):
    def __init__(self):
        super().__init__('yolo_logic_node')
        
        # Publisher to Mission Manager
        self.publisher_ = self.create_publisher(Twist, '/yolo_vel', 10)
        
        # Subscriptions
        self.subscription = self.create_subscription(ObjectsInfo, '/yolov5_ros2/object_detect', self.yolo_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
            
        # Parameters
        self.declare_parameter('min_score', 0.7)
        self.declare_parameter('show_video', True)
        self.min_score = self.get_parameter('min_score').get_parameter_value().double_value
        self.show_video = self.get_parameter('show_video').get_parameter_value().bool_value

        self.bridge = CvBridge()
        
        # Publisher for debug image to be viewed in rqt
        self.debug_pub = self.create_publisher(Image, '/yolo_logic_debug', 10)
        
        if self.show_video:
            self.image_sub = self.create_subscription(Image, '/yolov5_ros2/result_img', self.image_callback, 10)

        # State management
        self.state = "SEARCHING" # SEARCHING, TURNING_RIGHT, PARKING_FORWARD, STOPPED, TIMED_MANEUVER
        self.maneuver_end_time = 0.0
        self.current_sign = "None"
        
        # IMU variables for 90 degree turns
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.imu_received = False
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('YOLO Logic Node (IMU 90-deg & Auto Park) Started.')

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
            
            # Overlay current state logic on the video
            status = f"State: {self.state}"
            color = (0, 255, 0) if self.state == "SEARCHING" else (0, 0, 255)
            cv2.putText(cv_image, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            
            if self.state != "SEARCHING":
                cv2.putText(cv_image, f"Action: {self.current_sign}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            # Publish the image with text so it can be viewed in rqt
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.debug_pub.publish(debug_msg)

            cv2.imshow("YOLO Sign Detection & Logic", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'CV Bridge Error: {e}')

    def timer_callback(self):
        twist = Twist()
        
        if self.state == "TURNING_RIGHT":
            # Use IMU to turn exactly 90 degrees right
            error = self.normalize_angle(self.target_yaw - self.current_yaw)
            
            # Reached target heading
            if abs(error) < 0.08:
                if self.current_sign == "PARKING":
                    self.get_logger().info('Turn complete. Moving forward to park...')
                    self.state = "PARKING_FORWARD"
                    self.maneuver_end_time = time.time() + 1.2 # Move forward into parking spot for 1.2s
                else:
                    self.get_logger().info('90-Degree Turn Complete. Returning to Lane Keeping.')
                    self.state = "SEARCHING"
                    self.current_sign = "None"
                    self.publisher_.publish(Twist())
                return

            # Proportional Control for turn
            angular_vel = 0.8 * error
            min_vel = 0.25 
            if angular_vel > 0 and angular_vel < min_vel: angular_vel = min_vel
            if angular_vel < 0 and angular_vel > -min_vel: angular_vel = -min_vel
            if angular_vel > 0.6: angular_vel = 0.6
            if angular_vel < -0.6: angular_vel = -0.6
            
            twist.linear.x = 0.1 # Very slight forward motion during turn to prevent binding
            twist.angular.z = angular_vel
            self.publisher_.publish(twist)

        elif self.state == "PARKING_FORWARD":
            if time.time() < self.maneuver_end_time:
                twist.linear.x = 0.15 # Slow speed entering parking
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
            else:
                self.get_logger().info('Parked Successfully. Stopping.')
                self.state = "STOPPED"
                
        elif self.state == "STOPPED":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            
        elif self.state == "TIMED_MANEUVER":
            if time.time() < self.maneuver_end_time:
                twist.linear.x = 0.2
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
            else:
                self.get_logger().info('Straight Maneuver Complete.')
                self.state = "SEARCHING"
                self.current_sign = "None"
                self.publisher_.publish(Twist())

    def yolo_callback(self, msg):
        if self.state != "SEARCHING":
            return
            
        if not self.imu_received:
            self.get_logger().warn('Waiting for IMU data to perform maneuvers...', throttle_duration_sec=2.0)
            return

        for obj in msg.objects:
            if obj.score > self.min_score:
                self.get_logger().info(f'Detected: {obj.class_name}')
                
                if obj.class_name in ['R', 'turn_right']:
                    self.current_sign = "TURN RIGHT"
                    # -90 degrees (right turn)
                    self.target_yaw = self.normalize_angle(self.current_yaw - (math.pi / 2))
                    self.state = "TURNING_RIGHT"
                    break
                elif obj.class_name in ['P', 'parking', 'stop']:
                    self.current_sign = "PARKING"
                    # -90 degrees (right turn) into parking
                    self.target_yaw = self.normalize_angle(self.current_yaw - (math.pi / 2))
                    self.state = "TURNING_RIGHT"
                    break
                elif obj.class_name in ['S', 'go_straight']:
                    self.current_sign = "GO STRAIGHT"
                    self.maneuver_end_time = time.time() + 1.5
                    self.state = "TIMED_MANEUVER"
                    break

def main(args=None):
    rclpy.init(args=args)
    node = YoloLogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_video:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
