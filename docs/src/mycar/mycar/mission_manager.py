#!/usr/bin/env python3
# encoding: utf-8
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        
        # Output to the actual robot motors
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Input from Lane Keeping
        self.lane_sub = self.create_subscription(
            Twist, '/lane_vel', self.lane_callback, 10)
            
        # Input from Lidar Avoidance (Velocity & Status)
        self.avoid_vel_sub = self.create_subscription(
            Twist, '/avoid_vel', self.avoid_vel_callback, 10)
        self.avoid_status_sub = self.create_subscription(
            String, '/avoid_status', self.avoid_status_callback, 10)
            
        # Input from YOLO Sign Logic
        self.yolo_sub = self.create_subscription(
            Twist, '/yolo_vel', self.yolo_callback, 10)

        # Variables to store latest commands
        self.latest_lane_vel = Twist()
        self.latest_avoid_vel = Twist()
        self.latest_yolo_vel = Twist()
        self.avoid_status = "CLEAR" # CLEAR, AVOIDING, DANGER_STOP
        
        # State Management
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('FSD Mission Manager (New Version) Started. Priority: Safety > YOLO > Lane.')

    def lane_callback(self, msg):
        self.latest_lane_vel = msg

    def avoid_vel_callback(self, msg):
        self.latest_avoid_vel = msg

    def avoid_status_callback(self, msg):
        self.avoid_status = msg.data

    def yolo_callback(self, msg):
        self.latest_yolo_vel = msg

    def timer_callback(self):
        final_vel = Twist()
        
        # --- PRIORITY SYSTEM ---
        
        # 1. Highest Priority: Lidar Safety (Stop)
        if self.avoid_status == "DANGER_STOP":
            final_vel.linear.x = 0.0
            final_vel.angular.z = 0.0
            # self.get_logger().warn('MISSION: SAFETY STOP ACTIVE')
            
        # 2. High Priority: Active Avoidance (Dodge)
        elif self.avoid_status == "AVOIDING":
            final_vel = self.latest_avoid_vel
            # self.get_logger().info('MISSION: ACTIVE AVOIDANCE MODE')
            
        # 3. Medium Priority: YOLO Maneuvers (Signs)
        # If YOLO is sending a command (non-zero), use it
        elif abs(self.latest_yolo_vel.linear.x) > 0.01 or abs(self.latest_yolo_vel.angular.z) > 0.01:
            final_vel = self.latest_yolo_vel
            # self.get_logger().info('MISSION: YOLO MANEUVER ACTIVE')
            
        # 4. Default Priority: Lane Keeping
        else:
            final_vel = self.latest_lane_vel
            # self.get_logger().info('MISSION: LANE KEEPING MODE')

        # Publish the final decided velocity to the robot
        self.cmd_pub.publish(final_vel)

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
