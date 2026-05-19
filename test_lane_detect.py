import cv2
import numpy as np
import sys
import os

def test_lane_detection(image_path, use_white=True):
    # 1. Load Image
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: Could not read image at {image_path}")
        return

    # 2. Resize for Speed (match node logic)
    proc_w, proc_h = 320, 240
    img_resized = cv2.resize(img, (proc_w, proc_h))
    
    # 3. Single ROI (Bottom 40%)
    roi_top = int(proc_h * 0.6)
    roi = img_resized[roi_top:proc_h, :]
    
    # 4. HSV Masking
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Thresholds from lane_detect_node.py
    lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
    upper_yellow = np.array([40, 255, 255], dtype=np.uint8)
    lower_white = np.array([0, 0, 150], dtype=np.uint8)
    upper_white = np.array([180, 255, 255], dtype=np.uint8)

    if use_white:
        mask = cv2.inRange(hsv, lower_white, upper_white)
    else:
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
    # Fast noise removal
    kernel = np.ones((3,3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    # 5. Split L/R & Direct Moments
    mid_x = proc_w // 2
    mask_left = mask[:, :mid_x]
    mask_right = mask[:, mid_x:]
    
    M_l = cv2.moments(mask_left)
    M_r = cv2.moments(mask_right)
    
    # Require minimum area to consider valid
    cx_l = int(M_l['m10']/M_l['m00']) if M_l['m00'] > 100 else None
    cx_r = int(M_r['m10']/M_r['m00']) + mid_x if M_r['m00'] > 100 else None
    
    # 6. Logic
    center_pos = None
    last_lane_width = 160.0 # Default
    if cx_l is not None and cx_r is not None:
        last_lane_width = cx_r - cx_l
        center_pos = (cx_l + cx_r) / 2
    elif cx_l is not None:
        center_pos = cx_l + (last_lane_width / 2)
    elif cx_r is not None:
        center_pos = cx_r - (last_lane_width / 2)
        
    # 7. Visualization
    debug_img = img_resized.copy()
    roi_center_y = roi_top + ((proc_h - roi_top) // 2)
    
    if cx_l is not None:
        cv2.circle(debug_img, (cx_l, roi_center_y), 5, (0, 0, 255), -1)
    if cx_r is not None:
        cv2.circle(debug_img, (cx_r, roi_center_y), 5, (255, 0, 0), -1)
    if center_pos is not None:
        cv2.circle(debug_img, (int(center_pos), roi_center_y), 5, (0, 255, 255), -1)
        cv2.line(debug_img, (int(center_pos), roi_center_y), (mid_x, proc_h), (0, 255, 0), 2)
        
    error = center_pos - mid_x if center_pos is not None else 0.0
    cv2.putText(debug_img, f"Err: {error:.1f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    # Save Results
    output_path = "lane_debug_output.jpg"
    cv2.imwrite(output_path, debug_img)
    cv2.imwrite("mask_output.jpg", mask)
    print(f"Results saved to {output_path} and mask_output.jpg")
    print(f"Calculated Error: {error:.1f}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 test_lane_detect.py <image_path> [white/yellow]")
    else:
        path = sys.argv[1]
        mode = sys.argv[2] if len(sys.argv) > 2 else "white"
        test_lane_detection(path, use_white=(mode.lower() == "white"))
