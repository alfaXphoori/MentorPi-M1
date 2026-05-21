#!/usr/bin/env python3
# encoding: utf-8
"""
LaneKeepFullNode -- lane-centring controller with IMU-based 30-degree search.

Physical dimensions
  Lane  width : 30 cm
  Robot width : 18 cm
  -> clearance each side : 6 cm
  -> lane centre offset from one edge : 15 cm  (= lane_width / 2)

IMU search behaviour
  When the lane is lost for more than LOST_FRAMES_BEFORE_SEARCH frames,
  the robot stops and rotates by +/-30 degrees using the IMU yaw.
  Direction alternates L -> R -> L ... until the lane is found again.
  Maximum SEARCH_STEP_LIMIT steps before resetting to the original heading.
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu


# --------------------------------------------------------------------------- #
# Quaternion -> yaw helper
# --------------------------------------------------------------------------- #
def _yaw_from_quaternion(q):
    """Return yaw angle (radians) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# --------------------------------------------------------------------------- #
# Node
# --------------------------------------------------------------------------- #
class LaneKeepFullNode(Node):
    # ----- tuneable constants ----- #
    LANE_WIDTH_CM   = 30.0
    ROBOT_WIDTH_CM  = 18.0
    # ratio of lane width that = half-lane offset (always 0.5)
    HALF_LANE_RATIO = 0.5

    LOST_FRAMES_BEFORE_SEARCH = 8   # frames without detection -> start IMU search
    SEARCH_STEP_DEG           = 30.0  # degrees per IMU search step
    SEARCH_STEP_LIMIT         = 12   # max steps before giving up & resetting
    IMU_TURN_SPEED            = 0.70  # rad/s cap during IMU search
    IMU_REACH_TOLERANCE       = 0.05  # rad  (~3 deg) -> "close enough"

    def __init__(self):
        super().__init__('lane_keep_full')

        # ---- publishers / subscribers ---- #
        self.publisher_  = self.create_publisher(Twist, '/lane_vel', 10)
        self.debug_pub   = self.create_publisher(Image, '/lane_keep_full_debug', 10)

        self.cam_sub = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',
            self.image_callback,
            10,
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10
        )
        self.bridge = CvBridge()

        # ---- ROI layout: near -> far, higher weight near ---- #
        self.rois = [
            (0.80, 0.96, 0.0, 1.0, 0.50),
            (0.68, 0.80, 0.0, 1.0, 0.27),
            (0.56, 0.68, 0.0, 1.0, 0.15),
            (0.44, 0.56, 0.0, 1.0, 0.08),
        ]

        # ---- colour thresholds (LAB space, yellow lane) ---- #
        self.lower_yellow = np.array([0,   0, 145], dtype=np.uint8)
        self.upper_yellow = np.array([255, 255, 255], dtype=np.uint8)

        # ---- contour / lane-width params ---- #
        self.min_contour_area          = 120.0
        self.roi_min_area_scales       = [1.0, 0.85, 0.70, 0.55]
        self.lookahead_ratio           = 0.76
        self.default_lane_width_ratios = [0.42, 0.34, 0.26, 0.20]
        self.min_lane_width_ratio      = 0.12
        self.max_lane_width_ratio      = 0.85
        self.width_update_alpha        = 0.25

        # ---- speed params ---- #
        self.min_speed        = 0.07
        self.base_speed       = 0.14
        self.max_speed        = 0.22
        self.max_angular_z   = 0.90

        # ---- PD params ---- #
        self.kp = 0.008
        self.kd = 0.003
        self.target_smoothing = 0.35

        # ---- straight-line detection ---- #
        self.straight_error_threshold  = 0.08
        self.straight_path_threshold   = 0.10
        self.straight_frames_required  = 4

        # ---- runtime state ---- #
        self.smoothed_target_x     = None
        self.estimated_lane_widths = [None] * len(self.rois)
        self.last_error            = 0.0
        self.lost_lane_frames      = 0
        self.straight_frame_count  = 0

        # ---- IMU state ---- #
        self.imu_received        = False
        self.current_yaw         = 0.0
        self.search_target_yaw   = None   # None = not in mid-step
        self.search_direction    = 1.0    # +1 = left, -1 = right
        self.search_steps_done   = 0
        self.initial_yaw         = None   # yaw when search first started

        self.get_logger().info(
            f'LaneKeepFull started | Lane={self.LANE_WIDTH_CM}cm '
            f'Robot={self.ROBOT_WIDTH_CM}cm | IMU search {self.SEARCH_STEP_DEG}deg/step'
        )

    # ----------------------------------------------------------------------- #
    # IMU callback
    # ----------------------------------------------------------------------- #
    def imu_callback(self, msg: Imu):
        self.current_yaw  = _yaw_from_quaternion(msg.orientation)
        self.imu_received = True

    # ----------------------------------------------------------------------- #
    # Camera callback  (main loop)
    # ----------------------------------------------------------------------- #
    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = cv_image.shape

        mask          = self._create_mask(cv_image)
        debug_overlay = cv_image.copy()
        cv2.line(debug_overlay, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)

        detections, dual_roi_count = self._collect_detections(mask, debug_overlay, w, h)
        twist = Twist()

        if detections:
            # ---- lane found: normal tracking ---- #
            self.lost_lane_frames  = 0
            self.search_steps_done = 0
            self.search_target_yaw = None
            self.initial_yaw       = None

            raw_target_x, path_shift = self._estimate_target_x(detections, h, w, debug_overlay)
            self.smoothed_target_x   = self._smooth(raw_target_x)

            error            = self.smoothed_target_x - (w / 2.0)
            norm_error       = error / max(w / 2.0, 1.0)
            error_delta      = error - self.last_error

            # update preferred search direction for next loss event
            if error > 1.0:
                self.search_direction = -1.0
            elif error < -1.0:
                self.search_direction = 1.0

            straight = (
                dual_roi_count >= 2
                and abs(norm_error) < self.straight_error_threshold
                and path_shift    < self.straight_path_threshold
            )
            self.straight_frame_count = (self.straight_frame_count + 1) if straight else 0

            twist.linear.x  = self._compute_speed(norm_error, path_shift, len(detections), dual_roi_count)
            angular         = -(self.kp * error + self.kd * error_delta)
            twist.angular.z = float(np.clip(angular, -self.max_angular_z, self.max_angular_z))
            self.last_error = error

            # ---- debug drawing ---- #
            tp = (int(self.smoothed_target_x), int(h * self.lookahead_ratio))
            cv2.circle(debug_overlay, tp, 6, (0, 255, 0), -1)
            cv2.line(debug_overlay, (w // 2, h), tp, (0, 255, 0), 2)
            cv2.putText(debug_overlay,
                        f'ERR {int(error)}  SPD {twist.linear.x:.2f}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(debug_overlay,
                        f'ROIS {len(detections)}  BOTH {dual_roi_count}  STR {self.straight_frame_count}',
                        (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 0), 2)

        else:
            # ---- lane lost ---- #
            self.lost_lane_frames     += 1
            self.straight_frame_count  = 0

            if self.lost_lane_frames < self.LOST_FRAMES_BEFORE_SEARCH:
                # Brief grace period: stop and wait
                twist.linear.x  = 0.0
                twist.angular.z = 0.0
                cv2.putText(debug_overlay,
                            f'LANE LOST ({self.lost_lane_frames})',
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            else:
                # IMU 60-degree search
                twist = self._imu_search_step(debug_overlay)

        self.publisher_.publish(twist)

        mask_rgb      = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined_view = np.vstack((debug_overlay, mask_rgb))
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(combined_view, 'bgr8'))

    # ----------------------------------------------------------------------- #
    # IMU 60-degree search
    # ----------------------------------------------------------------------- #
    def _imu_search_step(self, debug) -> Twist:
        twist = Twist()

        if not self.imu_received:
            cv2.putText(debug, 'WAITING FOR IMU...',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return twist

        # Save heading at the very start of the search
        if self.initial_yaw is None:
            self.initial_yaw     = self.current_yaw
            self.search_direction = 1.0   # always start left

        # Set the next 60-deg target
        if self.search_target_yaw is None:
            step = self.search_direction * math.radians(self.SEARCH_STEP_DEG)
            self.search_target_yaw = _normalize_angle(self.current_yaw + step)
            self.get_logger().info(
                f'IMU search step {self.search_steps_done + 1}: '
                f'rotating {self.SEARCH_STEP_DEG * self.search_direction:+.0f}deg '
                f'-> target {math.degrees(self.search_target_yaw):.1f}deg'
            )

        error = _normalize_angle(self.search_target_yaw - self.current_yaw)

        if abs(error) < self.IMU_REACH_TOLERANCE:
            # Step complete
            self.search_steps_done += 1
            self.search_target_yaw  = None
            # Alternate direction: L -> R -> L -> ...
            self.search_direction  *= -1.0

            if self.search_steps_done >= self.SEARCH_STEP_LIMIT:
                self.get_logger().warn(
                    f'Max search steps ({self.SEARCH_STEP_LIMIT}) reached. Resetting.'
                )
                self.search_steps_done = 0
                self.search_direction  = 1.0
                self.initial_yaw       = None

            cv2.putText(debug, f'SEARCH STEP {self.search_steps_done} DONE',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return twist   # stop briefly between steps

        # Proportional rotation toward target
        angular_vel = float(np.clip(1.5 * error, -self.IMU_TURN_SPEED, self.IMU_TURN_SPEED))
        # Enforce minimum to overcome static friction
        if 0.0 < angular_vel < 0.25:
            angular_vel = 0.25
        elif -0.25 < angular_vel < 0.0:
            angular_vel = -0.25

        twist.linear.x  = 0.0
        twist.angular.z = angular_vel

        steps_left = self.SEARCH_STEP_LIMIT - self.search_steps_done
        cv2.putText(debug,
                    f'IMU SEARCH {self.SEARCH_STEP_DEG:.0f}deg  Step:{self.search_steps_done + 1}/{self.SEARCH_STEP_LIMIT}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(debug,
                    f'Target:{math.degrees(self.search_target_yaw):.1f}  Now:{math.degrees(self.current_yaw):.1f}  Left:{steps_left}',
                    (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 165, 255), 2)
        return twist

    # ----------------------------------------------------------------------- #
    # Lane mask
    # ----------------------------------------------------------------------- #
    def _create_mask(self, cv_image):
        blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
        lab     = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
        mask    = cv2.inRange(lab, self.lower_yellow, self.upper_yellow)
        k       = np.ones((5, 5), np.uint8)
        mask    = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
        mask    = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        return mask

    # ----------------------------------------------------------------------- #
    # Collect detections across ROIs
    # ----------------------------------------------------------------------- #
    def _collect_detections(self, mask, debug_overlay, w, h):
        detections     = []
        dual_roi_count = 0

        for idx, (y_s, y_e, x_s, x_e, weight) in enumerate(self.rois):
            y1, y2 = int(h * y_s), int(h * y_e)
            x1, x2 = int(w * x_s), int(w * x_e)
            roi_mask = mask[y1:y2, x1:x2]

            cv2.rectangle(debug_overlay, (x1, y1), (x2, y2), (255, 0, 0), 1)

            contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            candidates = []
            for c in contours:
                area = cv2.contourArea(c)
                if area < self._min_area(idx):
                    continue
                M = cv2.moments(c)
                if M['m00'] <= 0:
                    continue
                bx, by, bw, bh = cv2.boundingRect(c)
                candidates.append({
                    'cx': int(M['m10'] / M['m00']) + x1,
                    'cy': int(M['m01'] / M['m00']) + y1,
                    'area': area,
                    'bbox': (bx + x1, by + y1, bw, bh),
                })

            if not candidates:
                continue

            lane_info = self._select_edges(candidates, idx, w)
            if lane_info is None:
                continue

            self._draw_lane_info(debug_overlay, lane_info, idx + 1)
            detections.append({
                'cx': lane_info['lane_center'],
                'cy': lane_info['cy'],
                'weight': weight,
                'both_edges': lane_info['both_edges'],
            })
            if lane_info['both_edges']:
                dual_roi_count += 1

        return detections, dual_roi_count

    # ----------------------------------------------------------------------- #
    # Edge selection  --  lane centering with physical-dimension offset
    # ----------------------------------------------------------------------- #
    def _select_edges(self, candidates, roi_idx, image_width):
        sorted_c = sorted(candidates, key=lambda c: c['cx'])
        expected_width = self._get_expected_width(roi_idx, image_width)

        if len(sorted_c) >= 2:
            left  = sorted_c[0]
            right = sorted_c[-1]
            measured = right['cx'] - left['cx']
            min_w = image_width * self.min_lane_width_ratio
            max_w = image_width * self.max_lane_width_ratio

            if min_w <= measured <= max_w:
                updated = self._update_width(roi_idx, measured, image_width)
                return {
                    'left_edge':   left,
                    'right_edge':  right,
                    'lane_center': (left['cx'] + right['cx']) / 2.0,
                    'cy':          (left['cy'] + right['cy']) / 2.0,
                    'lane_width':  updated,
                    'both_edges':  True,
                }

        # Single edge -- use half lane-width as offset
        # Physical: robot must be at lane_centre = edge +/- lane_width_px/2
        primary = max(candidates, key=lambda c: c['area'])
        ref_cx  = self.smoothed_target_x if self.smoothed_target_x is not None else image_width / 2.0

        if primary['cx'] < ref_cx:
            # Left edge detected -> centre is to the right
            lane_center = primary['cx'] + expected_width * self.HALF_LANE_RATIO
            return {
                'left_edge':  primary,
                'right_edge': None,
                'lane_center': lane_center,
                'cy':          primary['cy'],
                'lane_width':  expected_width,
                'both_edges':  False,
            }
        else:
            # Right edge detected -> centre is to the left
            lane_center = primary['cx'] - expected_width * self.HALF_LANE_RATIO
            return {
                'left_edge':  None,
                'right_edge': primary,
                'lane_center': lane_center,
                'cy':          primary['cy'],
                'lane_width':  expected_width,
                'both_edges':  False,
            }

    # ----------------------------------------------------------------------- #
    # Lane width helpers
    # ----------------------------------------------------------------------- #
    def _min_area(self, idx):
        scale = self.roi_min_area_scales[min(idx, len(self.roi_min_area_scales) - 1)]
        return self.min_contour_area * scale

    def _get_expected_width(self, idx, image_width):
        w = self.estimated_lane_widths[idx]
        if w is not None:
            return w
        return self.default_lane_width_ratios[idx] * image_width

    def _update_width(self, idx, measured, image_width):
        clamped = float(np.clip(
            measured,
            image_width * self.min_lane_width_ratio,
            image_width * self.max_lane_width_ratio,
        ))
        prev = self.estimated_lane_widths[idx]
        updated = clamped if prev is None else (
            self.width_update_alpha * clamped + (1.0 - self.width_update_alpha) * prev
        )
        self.estimated_lane_widths[idx] = updated
        return updated

    # ----------------------------------------------------------------------- #
    # Target X estimation (polynomial fit across ROIs)
    # ----------------------------------------------------------------------- #
    def _estimate_target_x(self, detections, h, w, debug_overlay):
        xs = np.array([d['cx']     for d in detections], dtype=np.float32)
        ys = np.array([d['cy']     for d in detections], dtype=np.float32)
        ws = np.array([d['weight'] for d in detections], dtype=np.float32)

        if len(detections) == 1:
            return float(xs[0]), 1.0

        order   = np.argsort(ys)
        xs, ys, ws = xs[order], ys[order], ws[order]
        coeff   = np.polyfit(ys, xs, 1, w=ws)

        y_top    = int(h * self.rois[-1][0])
        y_bot    = int(h * self.rois[0][1])
        x_top    = int(np.clip(np.polyval(coeff, y_top), 0, w - 1))
        x_bot    = int(np.clip(np.polyval(coeff, y_bot), 0, w - 1))
        cv2.line(debug_overlay, (x_top, y_top), (x_bot, y_bot), (255, 255, 0), 2)

        target_y   = int(h * self.lookahead_ratio)
        target_x   = float(np.polyval(coeff, target_y))
        path_shift = abs(xs[-1] - xs[0]) / max(float(w), 1.0)
        return float(np.clip(target_x, 0, w - 1)), path_shift

    def _smooth(self, raw_x):
        if self.smoothed_target_x is None:
            return raw_x
        return (self.target_smoothing * raw_x
                + (1.0 - self.target_smoothing) * self.smoothed_target_x)

    # ----------------------------------------------------------------------- #
    # Speed computation
    # ----------------------------------------------------------------------- #
    def _compute_speed(self, norm_error, path_shift, det_count, dual_count):
        if det_count < 2 or dual_count == 0:
            return self.min_speed
        if dual_count >= 2 and self.straight_frame_count >= self.straight_frames_required:
            return self.max_speed
        turn_factor  = min(1.0, abs(norm_error) * 1.8 + path_shift * 2.2)
        cruise       = self.base_speed - (self.base_speed - self.min_speed) * turn_factor
        if dual_count == 1:
            cruise = min(cruise, self.base_speed - 0.02)
        return float(np.clip(cruise, self.min_speed, self.base_speed))

    # ----------------------------------------------------------------------- #
    # Debug drawing
    # ----------------------------------------------------------------------- #
    def _draw_lane_info(self, overlay, lane_info, label):
        ih, iw = overlay.shape[:2]
        for key, color, tag in (
            ('left_edge',  (0, 165, 255), 'L'),
            ('right_edge', (255, 0, 255), 'R'),
        ):
            edge = lane_info[key]
            if edge is None:
                continue
            bx, by, bw, bh = edge['bbox']
            cv2.rectangle(overlay, (bx, by), (bx + bw, by + bh), color, 2)
            cv2.circle(overlay, (int(edge['cx']), int(edge['cy'])), 5, color, -1)
            cv2.putText(overlay, f'{tag}{label}',
                        (int(edge['cx']) + 8, int(edge['cy']) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        lc = int(np.clip(lane_info['lane_center'], 0, iw - 1))
        ly = int(np.clip(lane_info['cy'],          0, ih - 1))
        cv2.circle(overlay, (lc, ly), 5, (0, 255, 0), -1)
        cv2.putText(overlay, f'C{label}', (lc + 8, ly + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        le = lane_info['left_edge']
        re = lane_info['right_edge']
        if le is not None and re is not None:
            cv2.line(overlay,
                     (int(le['cx']), int(le['cy'])),
                     (int(re['cx']), int(re['cy'])),
                     (0, 255, 255), 2)


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #
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
