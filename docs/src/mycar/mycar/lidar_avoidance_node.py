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
            
        # Publisher directly to motor controller for standalone running
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Declare and Get Parameters
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('stop_distance', 0.3)
        self.declare_parameter('forward_speed', 0.15)
        
        self.safe_distance = self.get_parameter('safe_distance').get_parameter_value().double_value
        self.stop_distance = self.get_parameter('stop_distance').get_parameter_value().double_value
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        
        self.is_obstacle_ahead = False
        
        self.get_logger().info('Lidar Avoidance Standalone Node Started.')

    def scan_callback(self, msg):
        # We focus on the front area (e.g., -30 to +30 degrees)
        # Assuming 0 degrees is straight ahead
        # LaserScan.ranges is an array. We need to find indices for the front.
        
        # Get the number of points
        num_points = len(msg.ranges)
        
        # Define front sector (in indices)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        # Calculate indices for -30 to 30 degrees
        front_indices = []
        for i in range(num_points):
            angle = angle_min + i * angle_increment
            if -0.5 < angle < 0.5: # ~ -28 to +28 degrees
                front_indices.append(i)
        
        if not front_indices:
            return

        # Get distances in the front sector
        front_ranges = [msg.ranges[i] for i in front_indices if msg.range_min < msg.ranges[i] < msg.range_max]
        
        if not front_ranges:
            return
            
        min_dist = min(front_ranges)
        
        twist = Twist()
        
        if min_dist < self.stop_distance:
            if not self.is_obstacle_ahead:
                self.get_logger().warn(f'EMERGENCY STOP! Obstacle at {min_dist:.2f}m')
                self.is_obstacle_ahead = True
            
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            
        elif min_dist < self.safe_distance:
            # Slow down
            self.get_logger().info(f'Slowing down. Obstacle at {min_dist:.2f}m')
            self.is_obstacle_ahead = False
            
            twist.linear.x = self.forward_speed / 2.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            
        else:
            if self.is_obstacle_ahead:
                self.get_logger().info('Path clear. Moving forward.')
                self.is_obstacle_ahead = False
                
            twist.linear.x = self.forward_speed
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
