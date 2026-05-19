import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition
import time

class CameraTiltNode(Node):
    def __init__(self):
        super().__init__('camera_tilt_node')
        self.publisher_ = self.create_publisher(ServosPosition, '/ros_robot_controller/bus_servo/set_position', 10)
        
        # Declare Parameters
        self.declare_parameter('servo_id', 2)
        self.declare_parameter('default_pos', 500)
        self.declare_parameter('min_pos', 300)
        self.declare_parameter('max_pos', 700)

        # Get Parameters
        self.servo_id = self.get_parameter('servo_id').get_parameter_value().integer_value
        self.current_pos = self.get_parameter('default_pos').get_parameter_value().integer_value
        self.min_pos = self.get_parameter('min_pos').get_parameter_value().integer_value
        self.max_pos = self.get_parameter('max_pos').get_parameter_value().integer_value
        
        self.get_logger().info(f'Camera Tilt Node Started (ID: {self.servo_id}). Sweeping...')
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.direction = 1

    def timer_callback(self):
        # Sweep logic: Move between min and max positions
        if self.current_pos >= self.max_pos:
            self.direction = -1
        elif self.current_pos <= self.min_pos:
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
