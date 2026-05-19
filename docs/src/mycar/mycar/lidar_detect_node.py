import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
import numpy as np

class LidarDetectNode(Node):
    def __init__(self):
        super().__init__('lidar_detect_node')
        
        # Subscriber to LiDAR data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            10)
            
        # Publishers for each direction (Consistent with avoidance logic)
        self.front_dist_pub = self.create_publisher(Float32, '/lidar_dist_front', 10)
        self.left_dist_pub = self.create_publisher(Float32, '/lidar_dist_left', 10)
        self.right_dist_pub = self.create_publisher(Float32, '/lidar_dist_right', 10)
        
        # Status Publisher
        self.status_pub = self.create_publisher(String, '/lidar_status', 10)
        
        # Parameters
        self.declare_parameter('stop_distance', 0.3)
        self.stop_dist = self.get_parameter('stop_distance').value
        
        self.get_logger().info('Lidar Detection Node Started. Monitoring Front, Left, and Right sectors.')

    def scan_callback(self, msg):
        num_points = len(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        # Initialize sector buffers
        front_sector = []
        left_sector = []
        right_sector = []
        
        for i in range(num_points):
            angle = angle_min + i * angle_increment
            dist = msg.ranges[i]
            
            if msg.range_min < dist < msg.range_max:
                # Center (Front) Sector: -15 to +15 degrees
                if -0.26 < angle < 0.26:
                    front_sector.append(dist)
                # Left Sector: +15 to +45 degrees
                elif 0.26 <= angle < 0.8:
                    left_sector.append(dist)
                # Right Sector: -45 to -15 degrees
                elif -0.8 < angle <= -0.26:
                    right_sector.append(dist)
        
        # Get minimum distances (default to max range if empty)
        min_front = min(front_sector) if front_sector else float(msg.range_max)
        min_left = min(left_sector) if left_sector else float(msg.range_max)
        min_right = min(right_sector) if right_sector else float(msg.range_max)
        
        # Publish distances
        self.front_dist_pub.publish(Float32(data=float(min_front)))
        self.left_dist_pub.publish(Float32(data=float(min_left)))
        self.right_dist_pub.publish(Float32(data=float(min_right)))
        
        # Determine and Publish Status
        status_msg = String()
        overall_min = min(min_front, min_left, min_right)
        
        if overall_min < self.stop_dist:
            status_msg.data = "OBSTACLE_NEAR"
        elif overall_min < self.stop_dist + 0.3:
            status_msg.data = "OBSTACLE_DETECTED"
        else:
            status_msg.data = "CLEAR"
            
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
