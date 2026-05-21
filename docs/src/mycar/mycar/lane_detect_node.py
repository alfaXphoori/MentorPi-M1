#!/usr/bin/env python3
# encoding: utf-8
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetectNode(Node):
    def __init__(self):
        super().__init__('lane_detect_node')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/lane_debug', 10)
        self.subscription = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # Yellow Line LAB Thresholds
        self.lower_yellow = np.array([0, 0, 145])
        self.upper_yellow = np.array([255, 255, 255])

        # Control Parameters
        self.base_speed = 0.10
        self.kp = 0.008
        self.kd = 0.003
        self.last_error = 0.0

        # Memory: learned half-lane-width (pixels from one edge to center)
        # Updated every time both L and R are visible
        self.half_lane_width = None
        self.half_width_alpha = 0.12  # EMA - slow stable update

        self.get_logger().info('Lane Detect Node Started (L+R center tracking).')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w, _ = cv_image.shape

            # ROI: bottom 40% of image
            roi_start = int(h * 0.60)
            roi = cv_image[roi_start:h, 0:w].copy()
            roi_h = roi.shape[0]

            # 1. Pre-process: blur -> LAB -> threshold
            blurred = cv2.GaussianBlur(roi, (5, 5), 0)
            lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
            mask = cv2.inRange(lab, self.lower_yellow, self.upper_yellow)
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # 2. Find all contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

            # Center reference line
            cv2.line(roi, (w // 2, 0), (w // 2, roi_h), (255, 0, 0), 1)

            twist = Twist()

            # 3. Filter valid contours by area and sort by cx
            candidates = []
            for c in contours:
                area = cv2.contourArea(c)
                if area < 200:
                    continue
                M = cv2.moments(c)
                if M['m00'] <= 0:
                    continue
                cx_c = int(M['m10'] / M['m00'])
                cy_c = int(M['m01'] / M['m00'])
                bx, by, bw, bh = cv2.boundingRect(c)
                candidates.append({'cx': cx_c, 'cy': cy_c, 'area': area, 'bbox': (bx, by, bw, bh)})

            if len(candidates) >= 2:
                # Sort by cx and take leftmost and rightmost
                candidates.sort(key=lambda c: c['cx'])
                left = candidates[0]
                right = candidates[-1]

                # Validate: gap must be reasonable (10%-85% of image width)
                gap = right['cx'] - left['cx']
                if w * 0.10 <= gap <= w * 0.85:
                    # Update half-lane-width memory
                    half = gap / 2.0
                    if self.half_lane_width is None:
                        self.half_lane_width = half
                    else:
                        self.half_lane_width = (self.half_width_alpha * half
                                                + (1 - self.half_width_alpha) * self.half_lane_width)

                    # Lane center = midpoint between L and R
                    lane_cx = int((left['cx'] + right['cx']) / 2)
                    lane_cy = int((left['cy'] + right['cy']) / 2)

                    error = lane_cx - w / 2
                    d_error = error - self.last_error
                    self.last_error = error

                    twist.linear.x = self.base_speed
                    twist.angular.z = float(np.clip(
                        -(self.kp * error + self.kd * d_error), -1.0, 1.0))

                    # Draw both edges and center
                    bx, by, bw2, bh = left['bbox']
                    cv2.rectangle(roi, (bx, by), (bx + bw2, by + bh), (0, 165, 255), 2)
                    cv2.circle(roi, (left['cx'], left['cy']), 6, (0, 165, 255), -1)
                    cv2.putText(roi, 'L', (left['cx'] + 8, left['cy']),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

                    bx, by, bw2, bh = right['bbox']
                    cv2.rectangle(roi, (bx, by), (bx + bw2, by + bh), (255, 0, 255), 2)
                    cv2.circle(roi, (right['cx'], right['cy']), 6, (255, 0, 255), -1)
                    cv2.putText(roi, 'R', (right['cx'] + 8, right['cy']),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

                    cv2.line(roi, (left['cx'], lane_cy), (right['cx'], lane_cy), (0, 255, 255), 2)
                    cv2.circle(roi, (lane_cx, lane_cy), 8, (0, 255, 0), -1)
                    cv2.line(roi, (w // 2, lane_cy), (lane_cx, lane_cy), (0, 255, 0), 2)
                    cv2.putText(roi, f'L+R  ERR:{int(error)}  hw:{int(self.half_lane_width)}px',
                                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            elif len(candidates) == 1 and self.half_lane_width is not None:
                # Single edge: infer center using memorized half-width
                edge = candidates[0]
                center_of_image = w / 2.0

                if edge['cx'] < center_of_image:
                    # Left edge visible -> center is to the right
                    lane_cx = int(edge['cx'] + self.half_lane_width)
                    side_label = 'L'
                    edge_color = (0, 165, 255)
                else:
                    # Right edge visible -> center is to the left
                    lane_cx = int(edge['cx'] - self.half_lane_width)
                    side_label = 'R'
                    edge_color = (255, 0, 255)

                error = lane_cx - w / 2
                d_error = error - self.last_error
                self.last_error = error

                twist.linear.x = self.base_speed * 0.7  # Slow down slightly
                twist.angular.z = float(np.clip(
                    -(self.kp * error + self.kd * d_error), -1.0, 1.0))

                # Draw edge and inferred center
                bx, by, bw2, bh = edge['bbox']
                cv2.rectangle(roi, (bx, by), (bx + bw2, by + bh), edge_color, 2)
                cv2.circle(roi, (edge['cx'], edge['cy']), 6, edge_color, -1)

                # Dashed line from edge to inferred center
                for i in range(6):
                    t0 = i / 6
                    t1 = (i + 0.5) / 6
                    x0 = int(edge['cx'] + (lane_cx - edge['cx']) * t0)
                    x1 = int(edge['cx'] + (lane_cx - edge['cx']) * t1)
                    cv2.line(roi, (x0, edge['cy']), (x1, edge['cy']), (0, 200, 255), 2)

                cv2.circle(roi, (lane_cx, edge['cy']), 8, (0, 200, 255), -1)
                cv2.putText(roi,
                            f'{side_label}-edge  ERR:{int(error)}  hw:{int(self.half_lane_width)}px',
                            (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2)
            else:
                # No lane found
                self.publisher_.publish(Twist())
                cv2.putText(roi, 'NO LANE', (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                combined_view = np.vstack((roi, mask_rgb))
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(combined_view, 'bgr8'))
                return

            self.publisher_.publish(twist)
            combined_view = np.vstack((roi, mask_rgb))
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(combined_view, 'bgr8'))

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
