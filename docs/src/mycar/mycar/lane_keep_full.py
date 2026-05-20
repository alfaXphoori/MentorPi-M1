#!/usr/bin/env python3
# encoding: utf-8
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


class LaneKeepFullNode(Node):
    def __init__(self):
        super().__init__('lane_keep_full')

        self.publisher_ = self.create_publisher(Twist, '/lane_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/lane_keep_full_debug', 10)
        self.subscription = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',
            self.image_callback,
            10,
        )
        self.bridge = CvBridge()

        # ROI layout: near ROI is weighted highest because it is most relevant for control.
        self.rois = [
            (0.80, 0.96, 0.0, 1.0, 0.55),
            (0.68, 0.80, 0.0, 1.0, 0.30),
            (0.56, 0.68, 0.0, 1.0, 0.15),
        ]

        self.lower_yellow = np.array([0, 0, 145], dtype=np.uint8)
        self.upper_yellow = np.array([255, 255, 255], dtype=np.uint8)

        self.min_contour_area = 120.0
        self.lookahead_ratio = 0.82
        self.default_lane_width_ratios = [0.42, 0.34, 0.26]
        self.min_lane_width_ratio = 0.12
        self.max_lane_width_ratio = 0.85
        self.width_update_alpha = 0.25

        self.min_speed = 0.07
        self.base_speed = 0.14
        self.max_speed = 0.22
        self.search_turn_speed = 0.45
        self.max_angular_speed = 0.90

        self.kp = 0.008
        self.kd = 0.003
        self.target_smoothing = 0.35

        self.straight_error_threshold = 0.08
        self.straight_path_threshold = 0.10
        self.straight_frames_required = 4
        self.search_swap_interval = 12

        self.smoothed_target_x = None
        self.estimated_lane_widths = [None] * len(self.rois)
        self.last_error = 0.0
        self.lost_lane_frames = 0
        self.straight_frame_count = 0
        self.search_direction = 1.0

        self.get_logger().info(
            'Lane Keep Full Node Started (left/right lane tracking, auto search, straight-line speed boost).'
        )

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = cv_image.shape

        mask = self.create_lane_mask(cv_image)
        debug_overlay = cv_image.copy()
        cv2.line(debug_overlay, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)

        detections, dual_roi_count = self.collect_roi_detections(mask, debug_overlay)
        twist = Twist()

        if detections:
            raw_target_x, path_shift = self.estimate_target_x(detections, h, w, debug_overlay)
            self.smoothed_target_x = self.smooth_target(raw_target_x)

            error = self.smoothed_target_x - (w / 2.0)
            normalized_error = error / max(w / 2.0, 1.0)
            error_delta = error - self.last_error

            if error > 1.0:
                self.search_direction = -1.0
            elif error < -1.0:
                self.search_direction = 1.0

            straight_segment = (
                dual_roi_count >= 2
                and abs(normalized_error) < self.straight_error_threshold
                and path_shift < self.straight_path_threshold
            )
            if straight_segment:
                self.straight_frame_count += 1
            else:
                self.straight_frame_count = 0

            twist.linear.x = self.compute_speed(
                normalized_error,
                path_shift,
                len(detections),
                dual_roi_count,
            )
            angular = -(self.kp * error + self.kd * error_delta)
            twist.angular.z = float(np.clip(angular, -self.max_angular_speed, self.max_angular_speed))

            self.last_error = error
            self.lost_lane_frames = 0

            target_point = (int(self.smoothed_target_x), int(h * self.lookahead_ratio))
            cv2.circle(debug_overlay, target_point, 6, (0, 255, 0), -1)
            cv2.line(debug_overlay, (w // 2, h), target_point, (0, 255, 0), 2)
            cv2.putText(
                debug_overlay,
                f'ERR {int(error)}  SPD {twist.linear.x:.2f}',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                debug_overlay,
                f'ROIS {len(detections)}  BOTH {dual_roi_count}  STRAIGHT {self.straight_frame_count}',
                (10, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 0),
                2,
            )
        else:
            self.lost_lane_frames += 1
            self.straight_frame_count = 0
            twist.linear.x = 0.0
            twist.angular.z = self.compute_search_turn()

            cv2.putText(
                debug_overlay,
                f'SEARCHING LANE {self.lost_lane_frames}',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )
            cv2.putText(
                debug_overlay,
                f'SEARCH TURN {twist.angular.z:.2f}',
                (10, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 165, 255),
                2,
            )

        self.publisher_.publish(twist)

        mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined_view = np.vstack((debug_overlay, mask_rgb))
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(combined_view, 'bgr8'))

    def create_lane_mask(self, cv_image):
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        lab_img = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
        mask = cv2.inRange(lab_img, self.lower_yellow, self.upper_yellow)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def collect_roi_detections(self, mask, debug_overlay):
        h, w = mask.shape
        detections = []
        dual_roi_count = 0

        for index, (y_s, y_e, x_s, x_e, weight) in enumerate(self.rois):
            y1, y2 = int(h * y_s), int(h * y_e)
            x1, x2 = int(w * x_s), int(w * x_e)
            roi_mask = mask[y1:y2, x1:x2]

            cv2.rectangle(debug_overlay, (x1, y1), (x2, y2), (255, 0, 0), 1)

            contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            candidates = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_contour_area:
                    continue

                moments = cv2.moments(contour)
                if moments['m00'] <= 0.0:
                    continue

                bx, by, bw, bh = cv2.boundingRect(contour)
                candidates.append({
                    'cx': int(moments['m10'] / moments['m00']) + x1,
                    'cy': int(moments['m01'] / moments['m00']) + y1,
                    'area': area,
                    'bbox': (bx + x1, by + y1, bw, bh),
                })

            if not candidates:
                continue

            lane_info = self.select_lane_edges(candidates, index, w)
            if lane_info is None:
                continue

            self.draw_lane_info(debug_overlay, lane_info, index + 1)
            detections.append({
                'cx': lane_info['lane_center'],
                'cy': lane_info['cy'],
                'weight': weight,
                'both_edges': lane_info['both_edges'],
            })
            if lane_info['both_edges']:
                dual_roi_count += 1

        return detections, dual_roi_count

    def select_lane_edges(self, candidates, roi_index, image_width):
        sorted_candidates = sorted(candidates, key=lambda item: item['cx'])
        expected_lane_width = self.get_expected_lane_width(roi_index, image_width)

        if len(sorted_candidates) >= 2:
            left_edge = sorted_candidates[0]
            right_edge = sorted_candidates[-1]
            measured_lane_width = right_edge['cx'] - left_edge['cx']
            if image_width * self.min_lane_width_ratio <= measured_lane_width <= image_width * self.max_lane_width_ratio:
                lane_width = self.update_lane_width(roi_index, measured_lane_width, image_width)
                return {
                    'left_edge': left_edge,
                    'right_edge': right_edge,
                    'lane_center': (left_edge['cx'] + right_edge['cx']) / 2.0,
                    'cy': (left_edge['cy'] + right_edge['cy']) / 2.0,
                    'lane_width': lane_width,
                    'both_edges': True,
                }

        primary_edge = max(candidates, key=lambda item: item['area'])
        if self.infer_single_edge_side(primary_edge['cx'], image_width) == 'left':
            lane_center = primary_edge['cx'] + expected_lane_width / 2.0
            return {
                'left_edge': primary_edge,
                'right_edge': None,
                'lane_center': lane_center,
                'cy': primary_edge['cy'],
                'lane_width': expected_lane_width,
                'both_edges': False,
            }

        lane_center = primary_edge['cx'] - expected_lane_width / 2.0
        return {
            'left_edge': None,
            'right_edge': primary_edge,
            'lane_center': lane_center,
            'cy': primary_edge['cy'],
            'lane_width': expected_lane_width,
            'both_edges': False,
        }

    def infer_single_edge_side(self, edge_x, image_width):
        reference_center = self.smoothed_target_x if self.smoothed_target_x is not None else image_width / 2.0
        return 'left' if edge_x < reference_center else 'right'

    def get_expected_lane_width(self, roi_index, image_width):
        lane_width = self.estimated_lane_widths[roi_index]
        if lane_width is not None:
            return lane_width
        return self.default_lane_width_ratios[roi_index] * image_width

    def update_lane_width(self, roi_index, measured_lane_width, image_width):
        clamped_width = float(np.clip(
            measured_lane_width,
            image_width * self.min_lane_width_ratio,
            image_width * self.max_lane_width_ratio,
        ))
        previous_width = self.estimated_lane_widths[roi_index]
        if previous_width is None:
            updated_width = clamped_width
        else:
            updated_width = (
                self.width_update_alpha * clamped_width
                + (1.0 - self.width_update_alpha) * previous_width
            )
        self.estimated_lane_widths[roi_index] = updated_width
        return updated_width

    def draw_lane_info(self, debug_overlay, lane_info, roi_label):
        image_h, image_w = debug_overlay.shape[:2]
        for edge_key, color, label in (
            ('left_edge', (0, 165, 255), 'L'),
            ('right_edge', (255, 0, 255), 'R'),
        ):
            edge = lane_info[edge_key]
            if edge is None:
                continue

            bx, by, bw, bh = edge['bbox']
            cv2.rectangle(debug_overlay, (bx, by), (bx + bw, by + bh), color, 2)
            cv2.circle(debug_overlay, (int(edge['cx']), int(edge['cy'])), 5, color, -1)
            cv2.putText(
                debug_overlay,
                f'{label}{roi_label}',
                (int(edge['cx']) + 8, int(edge['cy']) - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
            )

        lane_center = int(np.clip(lane_info['lane_center'], 0, image_w - 1))
        lane_cy = int(np.clip(lane_info['cy'], 0, image_h - 1))
        cv2.circle(debug_overlay, (lane_center, lane_cy), 5, (0, 255, 0), -1)
        cv2.putText(
            debug_overlay,
            f'C{roi_label}',
            (lane_center + 8, lane_cy + 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 0),
            1,
        )

        left_edge = lane_info['left_edge']
        right_edge = lane_info['right_edge']
        if left_edge is not None and right_edge is not None:
            cv2.line(
                debug_overlay,
                (int(left_edge['cx']), int(left_edge['cy'])),
                (int(right_edge['cx']), int(right_edge['cy'])),
                (0, 255, 255),
                2,
            )

    def estimate_target_x(self, detections, h, w, debug_overlay):
        xs = np.array([point['cx'] for point in detections], dtype=np.float32)
        ys = np.array([point['cy'] for point in detections], dtype=np.float32)
        weights = np.array([point['weight'] for point in detections], dtype=np.float32)

        if len(detections) == 1:
            target_x = float(xs[0])
            path_shift = 1.0
        else:
            order = np.argsort(ys)
            xs = xs[order]
            ys = ys[order]
            weights = weights[order]

            line_coeff = np.polyfit(ys, xs, 1, w=weights)
            y_top = int(h * self.rois[-1][0])
            y_bottom = int(h * self.rois[0][1])
            x_top = int(np.clip(np.polyval(line_coeff, y_top), 0, w - 1))
            x_bottom = int(np.clip(np.polyval(line_coeff, y_bottom), 0, w - 1))
            cv2.line(debug_overlay, (x_top, y_top), (x_bottom, y_bottom), (255, 255, 0), 2)

            target_y = int(h * self.lookahead_ratio)
            target_x = float(np.polyval(line_coeff, target_y))
            path_shift = abs(xs[-1] - xs[0]) / max(float(w), 1.0)

        return float(np.clip(target_x, 0, w - 1)), path_shift

    def smooth_target(self, raw_target_x):
        if self.smoothed_target_x is None:
            return raw_target_x
        return (
            self.target_smoothing * raw_target_x
            + (1.0 - self.target_smoothing) * self.smoothed_target_x
        )

    def compute_speed(self, normalized_error, path_shift, detection_count, dual_roi_count):
        if detection_count < 2:
            return self.min_speed

        if dual_roi_count == 0:
            return self.min_speed

        if dual_roi_count >= 2 and self.straight_frame_count >= self.straight_frames_required:
            return self.max_speed

        turn_factor = min(1.0, abs(normalized_error) * 1.8 + path_shift * 2.2)
        cruise_speed = self.base_speed - (self.base_speed - self.min_speed) * turn_factor
        if dual_roi_count == 1:
            cruise_speed = min(cruise_speed, self.base_speed - 0.02)
        return float(np.clip(cruise_speed, self.min_speed, self.base_speed))

    def compute_search_turn(self):
        if self.lost_lane_frames > 0 and self.lost_lane_frames % self.search_swap_interval == 0:
            self.search_direction *= -1.0
        return self.search_direction * self.search_turn_speed


def main(args=None):
    rclpy.init(args=args)
    node = LaneKeepFullNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
