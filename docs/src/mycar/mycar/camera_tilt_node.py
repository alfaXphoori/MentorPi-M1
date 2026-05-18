import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition
import time

class CameraTiltNode(Node):
    def __init__(self):
        super().__init__('camera_tilt_node')
        self.publisher_ = self.create_publisher(ServosPosition, '/ros_robot_controller/bus_servo/set_position', 10)
        
        # Parameters
        self.servo_id = 2  # Typically ID 2 is for Tilt (Up/Down)
        self.current_pos = 500 # Center position (0-1000 range)
        
        self.get_logger().info('Camera Tilt Node Started. Sweeping Up/Down...')
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.direction = 1

    def timer_callback(self):
        # Sweep logic: Move between 300 (Up) and 700 (Down)
        if self.current_pos >= 700:
            self.direction = -1
        elif self.current_pos <= 300:
            self.direction = 1
            
        self.current_pos += self.direction * 100
        self.set_tilt(self.current_pos)

    def set_tilt(self, position):
        msg = ServosPosition()
        msg.duration = 0.5 # Seconds to reach position
        
        servo = ServoPosition()
        servo.id = self.servo_id
        servo.position = int(position)
        
        msg.position = [servo]
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Setting Tilt to: {position}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraTiltNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
