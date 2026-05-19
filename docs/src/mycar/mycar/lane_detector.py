#!/usr/bin/env python3
# encoding: utf-8
import cv2
import numpy as np

class LaneDetector:
    def __init__(self):
        # Perspective Transform Setup (Warp to Bird's Eye View)
        self.src_pts = np.float32([
            [40, 240],   # Top Left
            [280, 240],  # Top Right
            [320, 280],  # Bottom Right
            [0, 280]     # Bottom Left
        ])
        self.dst_pts = np.float32([
            [0, 0], [320, 0], [320, 240], [0, 240]
        ])
        self.M = cv2.getPerspectiveTransform(self.src_pts, self.dst_pts)
        
        # Scanline heights for analysis
        self.scan_heights = [int(240 * 0.2), int(240 * 0.5), int(240 * 0.8)]

    def process_frame(self, frame):
        # 1. Resize and Warp
        img = cv2.resize(frame, (320, 240))
        warped = cv2.warpPerspective(img, self.M, (320, 240))
        
        # 2. LAB Color Extraction
        lab = cv2.cvtColor(warped, cv2.COLOR_BGR2LAB)
        l_channel, _, b_channel = cv2.split(lab)
        
        # White Lines (High L) and Yellow Lines (High b)
        _, white_mask = cv2.threshold(l_channel, 200, 255, cv2.THRESH_BINARY)
        _, yellow_mask = cv2.threshold(b_channel, 140, 255, cv2.THRESH_BINARY)
        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
        
        # 3. Multi-Scanline Analysis
        centers = []
        for h in self.scan_heights:
            line_data = combined_mask[h, :]
            moments = cv2.moments(line_data)
            if moments['m00'] > 0:
                centers.append(int(moments['m10'] / moments['m00']))
            else:
                centers.append(160) # Default to center
                
        # 4. Calculate Steering Error (Lookahead)
        # Weight further points more for stability
        target_center = (centers[2] * 0.3) + (centers[0] * 0.7)
        error = target_center - 160
        
        # 5. Debug Visualization
        debug_img = warped.copy()
        for i, h in enumerate(self.scan_heights):
            cv2.circle(debug_img, (centers[i], h), 5, (0, 255, 0), -1)
        cv2.line(debug_img, (160, 240), (int(target_center), self.scan_heights[0]), (0, 0, 255), 2)
        
        return error, debug_img, centers
