import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

class HandControlNode(Node):
    def __init__(self):
        super().__init__('hand_control_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        
        # Initialize MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils

        # Movement State
        self.is_moving = False
        self.current_twist = Twist()

        # Timer to publish command continuously at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Hand Control Node Started. 2 fingers: Start, 4-5 fingers: Stop.')

    def timer_callback(self):
        self.publisher_.publish(self.current_twist)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.hands.process(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(cv_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                fingers = self.count_fingers(hand_landmarks)
                self.get_logger().debug(f'Fingers detected: {fingers}')
                
                # Start: Exactly 2 fingers
                if fingers == 2:
                    if not self.is_moving:
                        self.get_logger().info('Gesture: 2 Fingers - START MOVING')
                        self.is_moving = True
                        self.current_twist.linear.x = 0.2
                
                # Stop: 4 or 5 fingers (more robust for "Stop")
                elif fingers >= 4:
                    if self.is_moving or self.current_twist.linear.x != 0.0:
                        self.get_logger().info(f'Gesture: {fingers} Fingers - STOP')
                        self.is_moving = False
                        self.current_twist.linear.x = 0.0

        cv2.imshow("Hand Control Feedback", cv_image)
        cv2.waitKey(1)

    def count_fingers(self, landmarks):
        fingers = []
        # Tip IDs: [Thumb, Index, Middle, Ring, Pinky]
        tip_ids = [4, 8, 12, 16, 20]
        
        # Thumb: Check if tip is further from palm than IP joint (simplified)
        # We use the IP joint (3) and the MCP joint (2) for comparison
        if landmarks.landmark[tip_ids[0]].x < landmarks.landmark[tip_ids[0] - 1].x:
            fingers.append(1)
        else:
            fingers.append(0)
            
        # 4 Fingers: Check if tip is above the PIP joint (tip.y < pip.y)
        for id in range(1, 5):
            if landmarks.landmark[tip_ids[id]].y < landmarks.landmark[tip_ids[id] - 2].y:
                fingers.append(1)
            else:
                fingers.append(0)
        
        return fingers.count(1)

def main(args=None):
    rclpy.init(args=args)
    node = HandControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
