#!/usr/bin/env python3
# encoding: utf-8
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .lane_detector import LaneDetector

class LaneDetectNode(Node):
    def __init__(self):
        super().__init__('lane_detect_node')
        
        # Parameters
        self.declare_parameter('base_speed', 0.15)
        self.declare_parameter('camera_topic', '/ascamera/camera_publisher/rgb0/image')
        
        # Core Algorithm
        self.detector = LaneDetector()
        self.bridge = CvBridge()
        
        # Publishers & Subscribers
        self.publisher_ = self.create_publisher(Twist, '/lane_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/lane_debug', 10)
        
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10)
            
        self.get_logger().info('Lane Detection Node (Modular) Started.')

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process using separate logic file
            error, debug_img, centers = self.detector.process_frame(cv_img)
            
            # Steering Logic
            twist = Twist()
            twist.linear.x = self.get_parameter('base_speed').value
            
            # Simple P control based on error and curvature
            short_center, mid_center, long_center = centers
            curvature_factor = abs(long_center - short_center) / 50.0
            twist.angular.z = -float(error) * (0.012 + curvature_factor * 0.005)
            
            self.publisher_.publish(twist)
            
            # Publish Debug Image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Lane Detection Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
