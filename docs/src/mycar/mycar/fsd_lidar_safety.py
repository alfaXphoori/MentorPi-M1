import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np

class FSDLidarSafety(Node):
    def __init__(self):
        super().__init__('fsd_lidar_safety')
        
        self.publisher_ = self.create_publisher(Twist, '/fsd/avoid_vel', 10)
        self.status_pub = self.create_publisher(String, '/fsd/safety_status', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)
        
        self.declare_parameter('max_speed', 0.12)
        self.declare_parameter('danger_dist', 0.1)
        self.declare_parameter('avoid_dist', 0.25)
        
        self.get_logger().info('FSD Lidar Safety Node Started.')

    def scan_callback(self, msg):
        num_points = len(msg.ranges)
        angle_min, angle_inc = msg.angle_min, msg.angle_increment
        left, right = [], []
        
        for i in range(num_points):
            angle = angle_min + i * angle_inc
            dist = msg.ranges[i]
            if msg.range_min < dist < msg.range_max:
                if 0 <= angle < 0.52: left.append(dist)
                elif -0.52 < angle < 0: right.append(dist)
        
        min_l = min(left) if left else 10.0
        min_r = min(right) if right else 10.0
        min_t = min(min_l, min_r)
        
        twist = Twist()
        status = "CLEAR"
        
        if min_t < self.get_parameter('danger_dist').value:
            status = "DANGER_STOP"
        elif min_t < self.get_parameter('avoid_dist').value:
            status = "AVOIDING"
            twist.linear.x = self.get_parameter('max_speed').value * 0.7
            if min_l < min_r: twist.angular.z = -0.7 # Turn Right
            else: twist.angular.z = 0.7 # Turn Left
            
        self.publisher_.publish(twist)
        self.status_pub.publish(String(data=status))

def main(args=None):
    rclpy.init(args=args)
    node = FSDLidarSafety()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
