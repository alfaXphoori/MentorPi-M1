#!/usr/bin/env python3
# encoding: utf-8
"""FSD Lane Keep Node - centre-of-lane tracking with IMU-based search.

Rules:
  - Drive forward ONLY when both L and R lane edges are detected.
  - If lane is lost, rotate by 45 degrees at a time using IMU to search.
  - After each 45-deg scan, check for lane. Repeat up to 4 times (180 deg total).
"""
import cv2
import math
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu


class FSDLaneKeep(Node):
    def __init__(self):
        super().__init__('fsd_lane_keep')

        # ---------- Publishers / Subscribers ----------
        self.publisher_ = self.create_publisher(Twist, '/fsd/lane_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/fsd/lane_debug', 10)
        self.subscription = self.create_subscription(
            Image, '/ascamera/camera_publisher/rgb0/image',
            self.image_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        self.bridge = CvBridge()

        # ---------- ROIs (near -> far) ----------
        # (y_start, y_end, x_start, x_end, weight)
        self.rois = [
            (0.80, 0.96, 0.0, 1.0, 0.65),   # Near
            (0.68, 0.80, 0.0, 1.0, 0.35),   # Mid
        ]

        # ---------- Colour thresholds (LAB - yellow) ----------
        self.lower_yellow = np.array([0, 0, 145], dtype=np.uint8)
        self.upper_yellow = np.array([255, 255, 255], dtype=np.uint8)

        # ---------- Lane-width estimation ----------
        self.min_contour_area = 120.0
        self.roi_area_scales = [1.0, 0.85]
        self.default_lane_width_ratios = [0.42, 0.34]
        self.min_lane_width_ratio = 0.12
        self.max_lane_width_ratio = 0.85
        self.width_ema_alpha = 0.25
        self.estimated_lane_widths = [None] * len(self.rois)

        # ---------- Control ----------
        self.lookahead_ratio = 0.86
        self.min_speed = 0.04
        self.base_speed = 0.12
        self.max_speed = 0.18
        self.max_angular_speed = 1.10
        self.imu_turn_speed = 0.80   # Angular speed used during IMU-based search

        # PID
        self.kp = 0.004
        self.kd = 0.006
        self.target_smoothing = 0.65

        self.straight_error_thresh = 0.08
        self.straight_path_thresh = 0.10
        self.straight_frames_required = 4

        # ---------- IMU State ----------
        self.current_yaw = 0.0
        self.imu_received = False

        # ---------- Search State Machine ----------
        # States: DRIVING | SEARCHING | CENTERING
        self.state = 'SEARCHING'         # Start in SEARCHING until L+R found
        self.search_target_yaw = None    # Target yaw for current 45-deg step
        self.search_steps_done = 0       # How many 45-deg steps done
        self.search_direction = 1.0      # +1 = left, -1 = right
        self.max_search_steps = 8        # 8 x 45 = 360 degrees max

        # ---------- Driving State ----------
        self.smoothed_target_x = None
        self.last_error = 0.0
        self.straight_frame_count = 0

        self.get_logger().info('FSD Lane Keep Node Started (IMU 45-deg search, L+R required).')

    # ------------------------------------------------------------------ #
    # IMU callback
    # ------------------------------------------------------------------ #
    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.imu_received = True

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # ------------------------------------------------------------------ #
    # Image callback - main loop
    # ------------------------------------------------------------------ #
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w, _ = cv_image.shape
            mask = self._create_mask(cv_image)
            debug = cv_image.copy()
            cv2.line(debug, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)

            detections, dual_count = self._detect_lanes(mask, debug)
            twist = Twist()

            # Check if we have BOTH L and R edges in at least 1 ROI
            has_both = dual_count >= 1

            # ---- STATE: SEARCHING (IMU 45-deg steps) ----
            if self.state in ('SEARCHING', 'CENTERING'):
                if has_both:
                    # Found L+R -> switch to CENTERING first
                    if self.state == 'SEARCHING':
                        self.state = 'CENTERING'
                        self.search_target_yaw = None
                        self.search_steps_done = 0
                        self.get_logger().info('Lane found (L+R). Centering before driving...')

                if self.state == 'CENTERING' and has_both:
                    # Compute error and center the robot before moving
                    raw_x, _ = self._estimate_target(detections, h, w, debug)
                    self.smoothed_target_x = self._smooth(raw_x)
                    error = self.smoothed_target_x - w / 2.0
                    norm_err = error / max(w / 2.0, 1.0)

                    angular = -(self.kp * error + self.kd * (error - self.last_error))
                    twist.linear.x = 0.0
                    twist.angular.z = float(np.clip(angular, -self.max_angular_speed, self.max_angular_speed))
                    self.last_error = error

                    # Switch to DRIVING when close to center
                    if abs(norm_err) < 0.10:
                        self.state = 'DRIVING'
                        self.get_logger().info('Centered! Starting to drive.')

                    cv2.putText(debug, f'CENTERING  ERR:{int(error)}',
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

                elif self.state == 'SEARCHING':
                    # No L+R seen -> rotate 45 degrees using IMU
                    twist = self._imu_search_step(debug)

            # ---- STATE: DRIVING (only when L+R detected) ----
            elif self.state == 'DRIVING':
                if has_both:
                    raw_x, path_shift = self._estimate_target(detections, h, w, debug)
                    self.smoothed_target_x = self._smooth(raw_x)

                    error = self.smoothed_target_x - w / 2.0
                    norm_err = error / max(w / 2.0, 1.0)
                    d_error = error - self.last_error

                    straight = (dual_count >= 1
                                and abs(norm_err) < self.straight_error_thresh
                                and path_shift < self.straight_path_thresh)
                    self.straight_frame_count = (self.straight_frame_count + 1) if straight else 0

                    twist.linear.x = self._speed(norm_err, path_shift, len(detections), dual_count)
                    angular = -(self.kp * error + self.kd * d_error)
                    twist.angular.z = float(np.clip(angular, -self.max_angular_speed, self.max_angular_speed))
                    self.last_error = error

                    # HUD
                    tp = (int(self.smoothed_target_x), int(h * self.lookahead_ratio))
                    cv2.circle(debug, tp, 6, (0, 255, 0), -1)
                    cv2.line(debug, (w // 2, h), tp, (0, 255, 0), 2)
                    cv2.putText(debug, f'DRIVING  ERR:{int(error)}  SPD:{twist.linear.x:.2f}',
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(debug, f'DUAL:{dual_count}  STR:{self.straight_frame_count}',
                                (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                else:
                    # Lost L+R while driving -> start searching again
                    self.state = 'SEARCHING'
                    self.search_target_yaw = None
                    self.search_steps_done = 0
                    self.straight_frame_count = 0
                    self.get_logger().warn('Lost L+R lane! Switching to SEARCHING...')
                    twist = self._imu_search_step(debug)

            self.publisher_.publish(twist)
            combined = np.vstack((debug, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(combined, 'bgr8'))

        except Exception as e:
            self.get_logger().error(f'FSD Lane Error: {e}')

    # ------------------------------------------------------------------ #
    # IMU 45-degree search step
    # ------------------------------------------------------------------ #
    def _imu_search_step(self, debug):
        twist = Twist()

        if not self.imu_received:
            cv2.putText(debug, 'WAITING FOR IMU...',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return twist

        # First call: set the 45-deg target
        if self.search_target_yaw is None:
            step = self.search_direction * (math.pi / 4)  # 45 degrees
            self.search_target_yaw = self.normalize_angle(self.current_yaw + step)
            self.get_logger().info(
                f'Search step {self.search_steps_done + 1}: rotating 45 deg to {math.degrees(self.search_target_yaw):.1f} deg')

        error = self.normalize_angle(self.search_target_yaw - self.current_yaw)

        # Reached the 45-deg target
        if abs(error) < 0.06:
            self.search_steps_done += 1
            self.search_target_yaw = None   # Trigger next step next frame

            # Alternate direction after every step: L, R, L, R ...
            self.search_direction *= -1.0

            if self.search_steps_done >= self.max_search_steps:
                self.get_logger().warn('Max search steps reached. Resetting.')
                self.search_steps_done = 0
                self.search_direction = 1.0

            cv2.putText(debug, f'SEARCH STEP {self.search_steps_done} DONE',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return twist  # Stop briefly between steps

        # Proportional control for rotation
        angular_vel = 1.2 * error
        if angular_vel > 0 and angular_vel < 0.25: angular_vel = 0.25
        if angular_vel < 0 and angular_vel > -0.25: angular_vel = -0.25
        if angular_vel > self.imu_turn_speed: angular_vel = self.imu_turn_speed
        if angular_vel < -self.imu_turn_speed: angular_vel = -self.imu_turn_speed

        twist.linear.x = 0.0
        twist.angular.z = angular_vel

        steps_left = self.max_search_steps - self.search_steps_done
        cv2.putText(debug,
                    f'SEARCHING (IMU 45deg) Step:{self.search_steps_done + 1} Left:{steps_left}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(debug,
                    f'Target:{math.degrees(self.search_target_yaw):.1f}  Now:{math.degrees(self.current_yaw):.1f}',
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)
        return twist

    # ------------------------------------------------------------------ #
    # Mask creation
    # ------------------------------------------------------------------ #
    def _create_mask(self, img):
        blurred = cv2.GaussianBlur(img, (5, 5), 0)
        lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
        mask = cv2.inRange(lab, self.lower_yellow, self.upper_yellow)
        k = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        return mask

    # ------------------------------------------------------------------ #
    # Dual-edge lane detection per ROI
    # ------------------------------------------------------------------ #
    def _detect_lanes(self, mask, debug):
        h, w = mask.shape
        detections = []
        dual_count = 0

        for idx, (y_s, y_e, x_s, x_e, weight) in enumerate(self.rois):
            y1, y2 = int(h * y_s), int(h * y_e)
            x1, x2 = int(w * x_s), int(w * x_e)
            roi = mask[y1:y2, x1:x2]
            cv2.rectangle(debug, (x1, y1), (x2, y2), (255, 0, 0), 1)

            contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            candidates = []
            min_area = self.min_contour_area * self.roi_area_scales[min(idx, len(self.roi_area_scales) - 1)]
            for c in contours:
                area = cv2.contourArea(c)
                if area < min_area:
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

            lane = self._select_edges(candidates, idx, w)
            if lane is None:
                continue

            self._draw_lane(debug, lane, idx + 1)
            detections.append({
                'cx': lane['center'],
                'cy': lane['cy'],
                'weight': weight,
                'both': lane['both'],
            })
            if lane['both']:
                dual_count += 1

        return detections, dual_count

    def _select_edges(self, candidates, roi_idx, img_w):
        sorted_c = sorted(candidates, key=lambda c: c['cx'])
        exp_w = self._expected_width(roi_idx, img_w)

        if len(sorted_c) >= 2:
            left, right = sorted_c[0], sorted_c[-1]
            measured = right['cx'] - left['cx']
            if img_w * self.min_lane_width_ratio <= measured <= img_w * self.max_lane_width_ratio:
                lw = self._update_width(roi_idx, measured, img_w)
                return {
                    'left': left, 'right': right,
                    'center': (left['cx'] + right['cx']) / 2.0,
                    'cy': (left['cy'] + right['cy']) / 2.0,
                    'width': lw, 'both': True,
                }

        edge = max(candidates, key=lambda c: c['area'])
        ref = self.smoothed_target_x if self.smoothed_target_x else img_w / 2.0
        if edge['cx'] < ref:
            center = edge['cx'] + exp_w / 2.0
            return {'left': edge, 'right': None, 'center': center,
                    'cy': edge['cy'], 'width': exp_w, 'both': False}
        else:
            center = edge['cx'] - exp_w / 2.0
            return {'left': None, 'right': edge, 'center': center,
                    'cy': edge['cy'], 'width': exp_w, 'both': False}

    # ------------------------------------------------------------------ #
    # Lane-width bookkeeping
    # ------------------------------------------------------------------ #
    def _expected_width(self, idx, img_w):
        w = self.estimated_lane_widths[idx]
        if w is not None:
            return w
        return self.default_lane_width_ratios[idx] * img_w

    def _update_width(self, idx, measured, img_w):
        clamped = float(np.clip(measured,
                                img_w * self.min_lane_width_ratio,
                                img_w * self.max_lane_width_ratio))
        prev = self.estimated_lane_widths[idx]
        updated = clamped if prev is None else (self.width_ema_alpha * clamped + (1 - self.width_ema_alpha) * prev)
        self.estimated_lane_widths[idx] = updated
        return updated

    # ------------------------------------------------------------------ #
    # Target estimation (weighted polyfit across ROIs)
    # ------------------------------------------------------------------ #
    def _estimate_target(self, detections, h, w, debug):
        xs = np.array([d['cx'] for d in detections], dtype=np.float32)
        ys = np.array([d['cy'] for d in detections], dtype=np.float32)
        ws = np.array([d['weight'] for d in detections], dtype=np.float32)

        if len(detections) == 1:
            return float(xs[0]), 1.0

        order = np.argsort(ys)
        xs, ys, ws = xs[order], ys[order], ws[order]
        coeffs = np.polyfit(ys, xs, 1, w=ws)

        y_top = int(h * self.rois[-1][0])
        y_bot = int(h * self.rois[0][1])
        x_top = int(np.clip(np.polyval(coeffs, y_top), 0, w - 1))
        x_bot = int(np.clip(np.polyval(coeffs, y_bot), 0, w - 1))
        cv2.line(debug, (x_top, y_top), (x_bot, y_bot), (255, 255, 0), 2)

        target_y = int(h * self.lookahead_ratio)
        target_x = float(np.polyval(coeffs, target_y))
        path_shift = abs(xs[-1] - xs[0]) / max(float(w), 1.0)
        return float(np.clip(target_x, 0, w - 1)), path_shift

    def _smooth(self, raw):
        if self.smoothed_target_x is None:
            return raw
        return self.target_smoothing * raw + (1 - self.target_smoothing) * self.smoothed_target_x

    # ------------------------------------------------------------------ #
    # Adaptive speed (only called in DRIVING state with both edges)
    # ------------------------------------------------------------------ #
    def _speed(self, norm_err, path_shift, n_det, dual_count):
        if dual_count >= 1 and self.straight_frame_count >= self.straight_frames_required:
            return self.max_speed
        turn = min(1.0, abs(norm_err) * 1.8 + path_shift * 2.2)
        spd = self.base_speed - (self.base_speed - self.min_speed) * turn
        return float(np.clip(spd, self.min_speed, self.base_speed))

    # ------------------------------------------------------------------ #
    # Debug drawing
    # ------------------------------------------------------------------ #
    def _draw_lane(self, debug, lane, label):
        h, w = debug.shape[:2]
        for key, colour, tag in (('left', (0, 165, 255), 'L'), ('right', (255, 0, 255), 'R')):
            edge = lane[key]
            if edge is None:
                continue
            bx, by, bw, bh = edge['bbox']
            cv2.rectangle(debug, (bx, by), (bx + bw, by + bh), colour, 2)
            cv2.circle(debug, (int(edge['cx']), int(edge['cy'])), 5, colour, -1)
            cv2.putText(debug, f'{tag}{label}', (int(edge['cx']) + 8, int(edge['cy']) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1)

        cx = int(np.clip(lane['center'], 0, w - 1))
        cy = int(np.clip(lane['cy'], 0, h - 1))
        cv2.circle(debug, (cx, cy), 5, (0, 255, 0), -1)
        cv2.putText(debug, f'C{label}', (cx + 8, cy + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        if lane['left'] is not None and lane['right'] is not None:
            cv2.line(debug,
                     (int(lane['left']['cx']), int(lane['left']['cy'])),
                     (int(lane['right']['cx']), int(lane['right']['cy'])),
                     (0, 255, 255), 2)


def main(args=None):
    rclpy.init(args=args)
    node = FSDLaneKeep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
