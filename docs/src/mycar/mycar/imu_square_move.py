import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math
import time

class ImuSquareMove(Node):
    def __init__(self):
        super().__init__('imu_square_move')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        
        self.current_yaw = 0.0
        self.imu_received = False
        
        self.forward_speed = 0.2
        self.turn_speed = 0.4
        self.drive_duration = 2.0
        
        self.get_logger().info('Waiting for IMU data...')
        
        # Wait for first IMU message
        while rclpy.ok() and not self.imu_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        self.run_square()

    def imu_callback(self, msg):
        # Convert quaternion to yaw
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.imu_received = True

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def run_square(self):
        self.get_logger().info('Starting IMU-based Square Movement...')
        
        # Use current yaw as starting heading
        target_yaw = self.current_yaw
        
        for i in range(4):
            # 1. Drive Forward while maintaining heading
            self.get_logger().info(f'Side {i+1}: Driving Forward...')
            self.move_forward(self.forward_speed, self.drive_duration, target_yaw)
            
            # 2. Turn 90 Degrees Right (Negative)
            self.get_logger().info(f'Side {i+1}: Turning 90° Right...')
            target_yaw = self.normalize_angle(target_yaw - (math.pi / 2))
            self.turn_to_yaw(target_yaw)
            
        self.get_logger().info('Square Complete. Stopping.')
        self.stop_robot()
        self.destroy_node()
        rclpy.shutdown()

    def move_forward(self, speed, duration, heading):
        msg = Twist()
        end_time = time.time() + duration
        while time.time() < end_time and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            msg.linear.x = speed
            
            # Use IMU to stay on track (Heading Correction)
            error = self.normalize_angle(heading - self.current_yaw)
            msg.angular.z = 0.8 * error  # P control for heading stability
            
            self.publisher_.publish(msg)
            time.sleep(0.05)

    def turn_to_yaw(self, target_yaw):
        msg = Twist()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            error = self.normalize_angle(target_yaw - self.current_yaw)
            
            if abs(error) < 0.03: # Accuracy threshold (radians, ~1.7 degrees)
                break
                
            # Proportional control for turning speed
            angular_vel = 0.7 * error
            # Clamp to max turn speed
            if angular_vel > self.turn_speed: angular_vel = self.turn_speed
            if angular_vel < -self.turn_speed: angular_vel = -self.turn_speed
            
            msg.angular.z = angular_vel
            self.publisher_.publish(msg)
            time.sleep(0.05)
        
        self.stop_robot()
        time.sleep(0.5) # Short pause after turn

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuSquareMove()

if __name__ == '__main__':
    main()
