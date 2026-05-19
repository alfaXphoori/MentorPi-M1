import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces.msg import ObjectsInfo # Assuming this is the message type from yolov5_ros2.md
import time

class YoloLogicNode(Node):
    def __init__(self):
        super().__init__('yolo_logic_node')
        
        # Publisher to Mission Manager
        self.publisher_ = self.create_publisher(Twist, '/yolo_vel', 10)
        
        # Subscribe to YOLOv5 detections
        self.subscription = self.create_subscription(
            ObjectsInfo,
            '/yolov5_ros2/object_detect',
            self.yolo_callback,
            10)
            
        # Declare and Get Parameters
        self.declare_parameter('turn_duration', 2.0)
        self.declare_parameter('stop_duration', 3.0)
        self.declare_parameter('min_score', 0.7)
        
        self.turn_duration = self.get_parameter('turn_duration').get_parameter_value().double_value
        self.stop_duration = self.get_parameter('stop_duration').get_parameter_value().double_value
        self.min_score = self.get_parameter('min_score').get_parameter_value().double_value

        # State management
        self.active_maneuver = False
        self.maneuver_end_time = 0.0
        self.maneuver_twist = Twist()
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('YOLO Logic Node (FSD Optimized) Started.')

    def timer_callback(self):
        if self.active_maneuver:
            if time.time() < self.maneuver_end_time:
                self.publisher_.publish(self.maneuver_twist)
            else:
                self.active_maneuver = False
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
                    self.start_maneuver(0.15, -0.6, self.turn_duration)
                    break
                elif obj.class_name in ['S', 'go_straight']:
                    self.start_maneuver(0.2, 0.0, 1.5)
                    break
                elif obj.class_name in ['P', 'parking', 'stop']:
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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
