import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces.msg import ObjectsInfo # Message type from yolov5_ros2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class YoloLogicNode(Node):
    def __init__(self):
        super().__init__('yolo_logic_node')
        
        # Publisher to Mission Manager
        self.publisher_ = self.create_publisher(Twist, '/yolo_vel', 10)
        
        # Subscribe to YOLOv5 detections (Bounding boxes & Classes)
        self.subscription = self.create_subscription(
            ObjectsInfo,
            '/yolov5_ros2/object_detect',
            self.yolo_callback,
            10)
            
        # Declare and Get Parameters
        self.declare_parameter('turn_duration', 2.0)
        self.declare_parameter('stop_duration', 3.0)
        self.declare_parameter('min_score', 0.7)
        self.declare_parameter('show_video', True)
        
        self.turn_duration = self.get_parameter('turn_duration').get_parameter_value().double_value
        self.stop_duration = self.get_parameter('stop_duration').get_parameter_value().double_value
        self.min_score = self.get_parameter('min_score').get_parameter_value().double_value
        self.show_video = self.get_parameter('show_video').get_parameter_value().bool_value

        # Video Display Setup
        self.bridge = CvBridge()
        self.debug_pub = self.create_publisher(Image, '/yolo_debug', 10)
        
        if self.show_video:
            # Subscribe to the annotated image from YOLO
            self.image_sub = self.create_subscription(
                Image,
                '/yolov5_ros2/result_img',
                self.image_callback,
                10)

        # State management
        self.active_maneuver = False
        self.maneuver_end_time = 0.0
        self.maneuver_twist = Twist()
        self.current_sign = "None"
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('YOLO Logic Node (FSD Optimized) Started. Video Feed: ON')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Overlay current state logic on the video
            status = f"State: {'MANEUVER' if self.active_maneuver else 'SEARCHING'}"
            color = (0, 0, 255) if self.active_maneuver else (0, 255, 0)
            cv2.putText(cv_image, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            
            if self.active_maneuver:
                cv2.putText(cv_image, f"Action: {self.current_sign}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            cv2.imshow("YOLO Sign Detection & Logic", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'CV Bridge Error: {e}')

    def timer_callback(self):
        if self.active_maneuver:
            if time.time() < self.maneuver_end_time:
                self.publisher_.publish(self.maneuver_twist)
            else:
                self.active_maneuver = False
                self.current_sign = "None"
                self.get_logger().info('Maneuver Complete. Returning to Lane Keeping.')
                # Publish zero to let mission manager know we are done
                self.publisher_.publish(Twist())

    def yolo_callback(self, msg):
        if self.active_maneuver:
            return

        for obj in msg.objects:
            if obj.score > self.min_score:
                self.get_logger().info(f'Detected: {obj.class_name}')
                
                if obj.class_name in ['R', 'turn_right']:
                    self.current_sign = "TURN RIGHT"
                    self.start_maneuver(0.15, -0.6, self.turn_duration)
                    break
                elif obj.class_name in ['S', 'go_straight']:
                    self.current_sign = "GO STRAIGHT"
                    self.start_maneuver(0.2, 0.0, 1.5)
                    break
                elif obj.class_name in ['P', 'parking', 'stop']:
                    self.current_sign = "STOP/PARKING"
                    self.start_maneuver(0.0, 0.0, self.stop_duration)
                    break

    def start_maneuver(self, linear_x, angular_z, duration):
        self.maneuver_twist.linear.x = linear_x
        self.maneuver_twist.angular.z = angular_z
        self.maneuver_end_time = time.time() + duration
        self.active_maneuver = True
        self.publisher_.publish(self.maneuver_twist)

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
