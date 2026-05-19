import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
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
            
        # Publisher for avoidance velocity (to be coordinated by Mission Manager)
        self.publisher_ = self.create_publisher(Twist, '/avoid_vel', 10)
        
        # Parameters
        self.declare_parameter('avoid_distance', 0.6)
        self.declare_parameter('stop_distance', 0.3)
        self.declare_parameter('max_speed', 0.15)
        
        self.avoid_dist = self.get_parameter('avoid_distance').value
        self.stop_dist = self.get_parameter('stop_distance').value
        self.max_speed = self.get_parameter('max_speed').value
        
        self.get_logger().info('Lidar Avoidance (Active Dodge) Node Started.')

    def scan_callback(self, msg):
        num_points = len(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        # Analyze sectors: Front-Left and Front-Right
        left_sector = []
        right_sector = []
        
        for i in range(num_points):
            angle = angle_min + i * angle_increment
            dist = msg.ranges[i]
            if msg.range_min < dist < msg.range_max:
                if 0 <= angle < 0.6:   # 0 to 35 degrees (Left)
                    left_sector.append(dist)
                elif -0.6 < angle < 0: # -35 to 0 degrees (Right)
                    right_sector.append(dist)
        
        twist = Twist()
        
        min_left = min(left_sector) if left_sector else 10.0
        min_right = min(right_sector) if right_sector else 10.0
        min_dist = min(min_left, min_right)
        
        if min_dist < self.stop_dist:
            # Too close to avoid, just stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn('Obstacle too close! Emergency Stop.')
        elif min_dist < self.avoid_dist:
            # Active Avoidance Logic
            twist.linear.x = self.max_speed * 0.7 # Slow down while dodging
            
            # If obstacle is more on the left, steer right (negative Z)
            # If obstacle is more on the right, steer left (positive Z)
            if min_left < min_right:
                self.get_logger().info(f'Dodge Right! (Dist: {min_left:.2f}m)')
                twist.angular.z = -0.5 # Steer Right
            else:
                self.get_logger().info(f'Dodge Left! (Dist: {min_right:.2f}m)')
                twist.angular.z = 0.5 # Steer Left
        else:
            # Path clear
            # Note: We publish zero to /avoid_vel so Mission Manager knows we don't need to dodge
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher_.publish(twist)

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
