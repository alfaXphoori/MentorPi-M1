#!/usr/bin/env python3
# encoding: utf-8
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class FSDMissionManager(Node):
    def __init__(self):
        super().__init__('fsd_mission_manager')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.lane_sub = self.create_subscription(Twist, '/fsd/lane_vel', self.lane_callback, 10)
        self.avoid_sub = self.create_subscription(Twist, '/fsd/avoid_vel', self.avoid_callback, 10)
        self.status_sub = self.create_subscription(String, '/fsd/safety_status', self.status_callback, 10)
        self.yolo_sub = self.create_subscription(Twist, '/yolo_vel', self.yolo_callback, 10) # Using existing yolo_vel

        self.lane_v, self.avoid_v, self.yolo_v = Twist(), Twist(), Twist()
        self.status = "CLEAR"
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('FSD Mission Manager Started. Control logic active.')

    def lane_callback(self, msg): self.lane_v = msg
    def avoid_callback(self, msg): self.avoid_v = msg
    def status_callback(self, msg): self.status = msg.data
    def yolo_callback(self, msg): self.yolo_v = msg

    def timer_callback(self):
        final = Twist()
        if self.status == "DANGER_STOP": pass
        elif self.status == "AVOIDING": final = self.avoid_v
        elif abs(self.yolo_v.linear.x) > 0.01 or abs(self.yolo_v.angular.z) > 0.01: final = self.yolo_v
        else: final = self.lane_v
        
        self.cmd_pub.publish(final)

def main(args=None):
    rclpy.init(args=args)
    node = FSDMissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
