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
            '/ascamera/camera_publisher/rgb0/image', # Change if your camera topic is different
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

        self.get_logger().info('Hand Control Node Started. 2 fingers: Forward, 5 fingers: Stop.')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.hands.process(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        
        twist = Twist()
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw landmarks for visual feedback
                self.mp_draw.draw_landmarks(cv_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Count fingers
                fingers = self.count_fingers(hand_landmarks)
                
                if fingers == 2:
                    self.get_logger().info('Gesture: 2 Fingers - Moving Forward')
                    twist.linear.x = 0.2
                elif fingers == 5:
                    self.get_logger().info('Gesture: 5 Fingers - STOP')
                    twist.linear.x = 0.0
                
                self.publisher_.publish(twist)

        # Optional: Show debug window (uncomment if running on robot with screen)
        # cv2.imshow("Hand Control Debug", cv_image)
        # cv2.waitKey(1)

    def count_fingers(self, landmarks):
        fingers = []
        # Tips of fingers (Thumb, Index, Middle, Ring, Pinky)
        tip_ids = [4, 8, 12, 16, 20]
        
        # Thumb (Check x coordinate for left/right hand)
        if landmarks.landmark[tip_ids[0]].x < landmarks.landmark[tip_ids[0] - 1].x:
            fingers.append(1)
        else:
            fingers.append(0)
            
        # 4 Fingers
        for id in range(1, 5):
            if landmarks.landmark[tip_ids[id]].y < landmarks.landmark[tip_ids[id] - 2].y:
                fingers.append(1)
            else:
                fingers.append(0)
        
        return fingers.count(1)

def main(args=None):
    rclpy.init(args=args)
    node = HandControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
