import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np

class LidarAvoidanceNode(Node):
    def __init__(self):
        super().__init__('lidar_avoidance_node')
        
        # Subscriber to LiDAR data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            10)
            
        # DIRECT PUBLISH to the robot motors
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/avoid_status', 10)
        
        # Parameters
        self.declare_parameter('max_speed', 0.15)
        self.declare_parameter('danger_dist', 0.3)
        self.declare_parameter('avoid_dist', 0.7)
        
        self.max_speed = self.get_parameter('max_speed').value
        self.danger_dist = self.get_parameter('danger_dist').value
        self.avoid_dist = self.get_parameter('avoid_dist').value
        
        self.get_logger().info('Lidar Avoidance Node (STANDALONE MODE) Started. Direct control of /cmd_vel.')

    def scan_callback(self, msg):
        num_points = len(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        left_ranges = []
        right_ranges = []
        
        for i in range(num_points):
            angle = angle_min + i * angle_increment
            dist = msg.ranges[i]
            
            if msg.range_min < dist < msg.range_max:
                if 0 <= angle < 0.52: # 0 to 30 deg (Left)
                    left_ranges.append(dist)
                elif -0.52 < angle < 0: # -30 to 0 deg (Right)
                    right_ranges.append(dist)
        
        min_left = min(left_ranges) if left_ranges else 10.0
        min_right = min(right_ranges) if right_ranges else 10.0
        min_total = min(min_left, min_right)
        
        twist = Twist()
        status = "CLEAR"
        
        if min_total < self.danger_dist:
            # STOP
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            status = "DANGER_STOP"
            # self.get_logger().warn('EMERGENCY STOP!')
            
        elif min_total < self.avoid_dist:
            # DODGE
            status = "AVOIDING"
            twist.linear.x = self.max_speed * 0.8
            
            if min_left < min_right:
                # Obstacle on left -> Turn Right
                turn_intensity = (self.avoid_dist - min_left) / self.avoid_dist
                twist.angular.z = -0.6 - (turn_intensity * 0.5)
            else:
                # Obstacle on right -> Turn Left
                turn_intensity = (self.avoid_dist - min_right) / self.avoid_dist
                twist.angular.z = 0.6 + (turn_intensity * 0.5)
        else:
            # CRUISE (Move forward by itself)
            twist.linear.x = self.max_speed
            twist.angular.z = 0.0
            status = "CLEAR"

        # Send command DIRECTLY to the motors
        self.publisher_.publish(twist)
        self.status_pub.publish(String(data=status))

def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
