import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquareMove(Node):
    def __init__(self):
        super().__init__('square_move')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.forward_speed = 0.2
        self.turn_speed = 0.5
        self.drive_duration = 2.0
        self.turn_duration = 1.5
        
        self.run_square()

    def run_square(self):
        self.get_logger().info('Starting Square Movement...')
        for i in range(4):
            self.get_logger().info(f'Driving Side {i+1}...')
            self.move(self.forward_speed, 0.0, self.drive_duration)
            self.get_logger().info(f'Turning Right...')
            self.move(0.0, -self.turn_speed, self.turn_duration)
        self.get_logger().info('Square Complete. Stopping.')
        self.move(0.0, 0.0, 1.0)
        self.destroy_node()
        rclpy.shutdown()

    def move(self, linear_x, angular_z, duration):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(msg)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = SquareMove()

if __name__ == '__main__':
    main()
