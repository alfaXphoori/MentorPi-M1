#!/usr/bin/env python3
# encoding: utf-8
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces.msg import ObjectsInfo # Custom message from yolov5_ros2
import time

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        
        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to Lane Detection output (we will modify lane_detect_node to publish to a sub-topic)
        self.lane_sub = self.create_subscription(
            Twist, '/lane_vel', self.lane_callback, 10)
            
        # Subscribe to YOLOv5 detections
        self.yolo_sub = self.create_subscription(
            ObjectsInfo, '/yolov5_ros2/object_detect', self.yolo_callback, 10)
            
        # Parameters
        self.declare_parameter('min_confidence', 0.6)
        
        # State Management
        self.current_state = "LANE_FOLLOWING" # LANE_FOLLOWING, MANEUVER, STOPPED
        self.maneuver_end_time = 0.0
        self.maneuver_twist = Twist()
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Mission Manager (FSD) Started. Ready for autonomous driving.')

    def lane_callback(self, msg):
        # Only use lane commands if we are in LANE_FOLLOWING state
        if self.current_state == "LANE_FOLLOWING":
            self.cmd_pub.publish(msg)

    def yolo_callback(self, msg):
        min_conf = self.get_parameter('min_confidence').value
        
        for obj in msg.objects:
            if obj.score > min_conf:
                self.process_detection(obj.class_name)
                break # Process only the highest priority/first detected object

    def process_detection(self, label):
        # Prevent new maneuvers if already doing one (except STOP)
        if self.current_state == "MANEUVER" and label != "red_light":
            return

        self.get_logger().info(f'Processing Sign: {label}')
        
        if label == "red_light":
            self.current_state = "STOPPED"
            self.stop_robot()
            
        elif label == "green_light":
            if self.current_state == "STOPPED":
                self.current_state = "LANE_FOLLOWING"
                self.get_logger().info('Green light! Resuming...')

        elif label in ["R", "turn_right"]:
            self.start_maneuver(0.15, -0.7, 2.5) # Speed, Turn, Duration
            
        elif label in ["S", "go_straight"]:
            self.start_maneuver(0.2, 0.0, 1.5)
            
        elif label in ["P", "parking", "stop"]:
            self.start_maneuver(0.0, 0.0, 3.0) # Stop for 3 seconds

    def start_maneuver(self, x, z, duration):
        self.maneuver_twist.linear.x = x
        self.maneuver_twist.angular.z = z
        self.maneuver_end_time = time.time() + duration
        self.current_state = "MANEUVER"
        self.get_logger().info(f'Starting Maneuver for {duration}s')

    def stop_robot(self):
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)

    def timer_callback(self):
        if self.current_state == "MANEUVER":
            if time.time() < self.maneuver_end_time:
                self.cmd_pub.publish(self.maneuver_twist)
            else:
                self.current_state = "LANE_FOLLOWING"
                self.get_logger().info('Maneuver finished. Returning to Lane Following.')
        
        elif self.current_state == "STOPPED":
            self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
