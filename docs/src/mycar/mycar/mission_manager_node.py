import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import time
import math

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        
        # Subscriptions
        self.lane_sub = self.create_subscription(Twist, '/lane_vel', self.lane_callback, 10)
        self.yolo_sub = self.create_subscription(Twist, '/yolo_vel', self.yolo_callback, 10)
        self.lidar_sub = self.create_subscription(Twist, '/lidar_vel', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        # Publishers
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        
        # Control Variables
        self.lane_cmd = Twist()
        self.yolo_cmd = Twist()
        self.lidar_cmd = Twist()
        
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.imu_received = False
        
        self.current_status = "IDLE"
        self.yolo_active_until = 0.0
        self.lidar_active = False
        
        # Timestamps
        self.last_lane_time = 0.0
        self.last_yolo_time = 0.0
        self.last_lidar_time = 0.0
        
        # Main Control Loop (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Mission Manager (FSD Brain + IMU + Safety) Started.')

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.imu_received = True

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def lane_callback(self, msg):
        self.lane_cmd = msg
        self.last_lane_time = time.time()

    def yolo_callback(self, msg):
        # Update timestamp
        self.last_yolo_time = time.time()
        self.yolo_cmd = msg
        
        # Check if YOLO wants to start a maneuver
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            # If not already in an active maneuver, initialize it
            if time.time() > self.yolo_active_until:
                # If it's a turn maneuver, set IMU target
                if abs(msg.angular.z) > 0.3:
                    direction = -1.0 if msg.angular.z < 0 else 1.0
                    self.target_yaw = self.normalize_angle(self.current_yaw + (direction * math.pi / 2))
                    self.get_logger().info(f'YOLO: New Maneuver Detected. Setting IMU Target: {self.target_yaw:.2f}')
            
            # Keep maneuver active
            self.yolo_active_until = time.time() + 0.5

    def lidar_callback(self, msg):
        self.lidar_cmd = msg
        self.last_lidar_time = time.time()
        # Lidar override is active if it commands a stop
        if abs(msg.linear.x) < 0.01:
            self.lidar_active = True
        else:
            self.lidar_active = False

    def control_loop(self):
        now = time.time()
        final_cmd = Twist()
        
        # --- PRIORITY 1: LIDAR SAFETY (RESTORED TO TOP) ---
        if self.lidar_active and (now - self.last_lidar_time < 0.5):
            self.current_status = "SAFETY_OVERRIDE"
            final_cmd = self.lidar_cmd
            
        # --- PRIORITY 2: YOLO MANEUVERS (WITH IMU STABILIZATION) ---
        elif now < self.yolo_active_until and (now - self.last_yolo_time < 0.5):
            self.current_status = "YOLO_MANEUVER"
            final_cmd = self.yolo_cmd
            
            # IMU Assistance for turns
            if self.imu_received and abs(self.yolo_cmd.angular.z) > 0.1:
                error = self.normalize_angle(self.target_yaw - self.current_yaw)
                if abs(error) < 0.05: # Target reached
                    final_cmd.angular.z = 0.0
                    self.yolo_active_until = 0.0 # Exit maneuver
                    self.get_logger().info('YOLO Maneuver: IMU Target Reached.')
                else:
                    # P-Control for turning
                    final_cmd.angular.z = 0.8 * error
                    # Clamp to max speed from original command
                    max_speed = abs(self.yolo_cmd.angular.z)
                    final_cmd.angular.z = max(min(final_cmd.angular.z, max_speed), -max_speed)

        # --- PRIORITY 3: LANE KEEPING ---
        elif (now - self.last_lane_time < 0.5):
            self.current_status = "LANE_KEEPING"
            final_cmd = self.lane_cmd
            
        else:
            self.current_status = "NO_INPUT_STOP"
            final_cmd = Twist()
            
        # Publish commands
        self.publisher_.publish(final_cmd)
        
        # Publish status
        status_msg = String()
        status_msg.data = self.current_status
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
