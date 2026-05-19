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
            
        # Publisher for avoidance velocity (Target: Mission Manager)
        self.publisher_ = self.create_publisher(Twist, '/avoid_vel', 10)
        self.status_pub = self.create_publisher(String, '/avoid_status', 10)
        
        # Parameters for Avoidance
        self.declare_parameter('max_speed', 0.12)
        self.declare_parameter('danger_dist', 0.3)    # Stop and wait
        self.declare_parameter('avoid_dist', 0.6)     # Start dodging
        
        self.max_speed = self.get_parameter('max_speed').value
        self.danger_dist = self.get_parameter('danger_dist').value
        self.avoid_dist = self.get_parameter('avoid_dist').value
        
        self.get_logger().info('Lidar Avoidance Node (Active Dodge) Started.')

    def scan_callback(self, msg):
        num_points = len(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        # Sectors for dodging (Front-Left: 0 to 30, Front-Right: -30 to 0)
        left_ranges = []
        right_ranges = []
        
        for i in range(num_points):
            angle = angle_min + i * angle_increment
            dist = msg.ranges[i]
            
            if msg.range_min < dist < msg.range_max:
                # Left sector (0 to 0.52 rad / 0 to 30 deg)
                if 0 <= angle < 0.52:
                    left_ranges.append(dist)
                # Right sector (-0.52 to 0 rad / -30 to 0 deg)
                elif -0.52 < angle < 0:
                    right_ranges.append(dist)
        
        # Calculate minimum distances per sector
        min_left = min(left_ranges) if left_ranges else 10.0
        min_right = min(right_ranges) if right_ranges else 10.0
        min_total = min(min_left, min_right)
        
        twist = Twist()
        status = "CLEAR"
        
        if min_total < self.danger_dist:
            # DANGER: Stop immediately
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            status = "DANGER_STOP"
            self.get_logger().warn(f'DANGER! Obstacle at {min_total:.2f}m. STOPPING.')
            
        elif min_total < self.avoid_dist:
            # AVOID: Steer away from the closer obstacle
            status = "AVOIDING"
            twist.linear.x = self.max_speed * 0.6 # Slow down for safety
            
            # Logic: If left is blocked, turn right (negative Z)
            # If right is blocked, turn left (positive Z)
            # The closer the obstacle, the sharper the turn
            diff = min_left - min_right
            
            if min_left < min_right:
                # Obstacle is more on the left -> Turn Right
                # Calculate turn intensity based on closeness
                turn_strength = (self.avoid_dist - min_left) / self.avoid_dist
                twist.angular.z = -0.5 - turn_strength # Base turn + extra
                self.get_logger().info(f'Dodge RIGHT (L:{min_left:.2f}m R:{min_right:.2f}m)')
            else:
                # Obstacle is more on the right -> Turn Left
                turn_strength = (self.avoid_dist - min_right) / self.avoid_dist
                twist.angular.z = 0.5 + turn_strength
                self.get_logger().info(f'Dodge LEFT (L:{min_left:.2f}m R:{min_right:.2f}m)')
        else:
            # CLEAR: No need for avoidance input
            # We publish zero velocity so Mission Manager knows avoidance is idle
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            status = "CLEAR"

        # Publish
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
