import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.1
        self.publisher_.publish(msg)
        self.get_logger().info('My car is driving...')

def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
