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

        # Movement State (Default to moving)
        self.is_moving = True
        self.current_twist = Twist()
        self.current_twist.linear.x = 0.2

        # Timer to publish command continuously at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Hand Control Node Started. Default: Moving. 5 fingers: Stop.')

    def timer_callback(self):
        self.publisher_.publish(self.current_twist)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image = cv2.flip(cv_image, 1) # Mirror view
        results = self.hands.process(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        
        # Default behavior in this callback: Assume we keep moving
        # unless we specifically see the stop gesture
        fingers = -1 # -1 means no hand detected
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(cv_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                fingers = self.count_fingers(hand_landmarks)
                
                # If 5 fingers are detected (Stop gesture)
                if fingers >= 5:
                    self.is_moving = False
                    self.current_twist.linear.x = 0.0
                else:
                    # Any other gesture: Keep moving
                    self.is_moving = True
                    self.current_twist.linear.x = 0.2
        else:
            # No hand detected: Keep moving (as requested)
            self.is_moving = True
            self.current_twist.linear.x = 0.2

        # Draw status on image
        display_text = f"Fingers: {fingers}" if fingers != -1 else "No Hand"
        cv2.putText(cv_image, display_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        status = "STOPPED (5 Fingers)" if not self.is_moving else "MOVING (Default)"
        color = (0, 0, 255) if not self.is_moving else (0, 255, 0)
        cv2.putText(cv_image, f"Status: {status}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        cv2.imshow("Hand Control Feedback", cv_image)
        cv2.waitKey(1)

    def count_fingers(self, landmarks):
        fingers = []
        tip_ids = [4, 8, 12, 16, 20]
        
        # 4 Fingers
        for id in range(1, 5):
            if landmarks.landmark[tip_ids[id]].y < landmarks.landmark[tip_ids[id] - 2].y:
                fingers.append(1)
            else:
                fingers.append(0)
        
        # Thumb
        if abs(landmarks.landmark[tip_ids[0]].x - landmarks.landmark[5].x) > abs(landmarks.landmark[tip_ids[0] - 1].x - landmarks.landmark[5].x):
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
