# MentorPi M1 robot Mastering Path: Full Self-Driving (FSD) Engineering

Welcome to the MentorPi M1 robot FSD engineering track. This specialized learning path discards general robotics concepts to focus strictly on building a **Full Self-Driving (FSD) Autonomous Vehicle**. You will learn to integrate motion control, sensor fusion, computer vision, and AI.

---

## 🟢 Phase 1: Basic Motion Control
**Objective:** Establishing communication with the hardware and mastering timed and sensor-based movement sequences.

### 1.1 drive_node.py (The "Hello World" of Movement)
Learn the basics of publishing to the `/cmd_vel` topic to make the robot move.
*   **Key Concept:** `geometry_msgs/Twist` message structure.
*   **Logic:** Continuous publication of linear and angular velocity.

### 1.2 square_move.py (Timed Sequences)
Build on movement by adding time-based logic to perform a specific pattern.
*   **Key Concept:** Using `time.sleep()` or timers to manage movement durations.
*   **Logic:** Drive forward for X seconds, then turn for Y seconds, repeated 4 times.

### 1.3 imu_square_move.py (Precision with Sensors)
Move from timed guesses to sensor-based precision using the Inertial Measurement Unit (IMU).
*   **Key Concept:** Feedback loops and quaternion-to-yaw conversion.
*   **Logic:** Use IMU yaw data to maintain a straight heading and perform exact 90-degree turns.

---

## 🟡 Phase 2: Computer Vision Foundations
**Objective:** Equip the vehicle with "eyes" to interact with the environment and stay on the road.

### 2.1 color_control.py (Reaction to Color)
The first step in vision: making movement decisions based on detected colors.
*   **Key Concept:** OpenCV HSV color space and Region of Interest (ROI) cropping.
*   **Logic:** Detect GREEN to start moving, RED to stop.

### 2.2 lane_detect_node.py (Basic Lane Following)
Detect and follow a yellow lane markings using a single ROI.
*   **Key Concept:** LAB Color Space, Image Moments (Centroid), and Proportional Control (P-Control).
*   **Logic:** Find the center of the yellow line and adjust steering (`angular.z`) to keep it centered.

### 2.3 lane_keep_node.py (Advanced Multi-ROI Tracking)
An upgrade to lane detection using three independent ROIs for smoother curve handling.
*   **Key Concept:** Weighted averaging of multiple look-ahead points.
*   **Logic:** Split the view into Near, Mid, and Far zones. Near zone keeps the car straight; Far zone prepares for upcoming turns.

---

## 🟠 Phase 3: Spatial Safety & AI
**Objective:** Implement collision avoidance and interpret complex environmental symbols.

### 3.1 lidar_detect_node.py (Distance Monitoring)
Learn to process 2D LiDAR data to "feel" the distance to objects in front of the car.
*   **Key Concept:** `sensor_msgs/LaserScan` processing and sector analysis.
*   **Logic:** Monitor the -30 to +30 degree front sector and report status (CLEAR/DETECTED/NEAR).

### 3.2 lidar_avoidance_node.py (Active Dodging)
Move beyond stopping—teach the robot to actively steer away from obstacles.
*   **Key Concept:** Vector-based avoidance logic.
*   **Logic:** Compare distances on the Left vs. Right side of the front sector. If an obstacle is on the left, steer right automatically.

### 3.3 yolo_logic_node.py (Traffic Sign Navigation)
Integrate Deep Learning (YOLOv5) to make routing decisions based on physical signs.
*   **Key Concept:** AI Inference integration and state-machine maneuvers.
*   **Logic:** Interpret signs like 'R' (Turn Right), 'P' (Parking), or 'S' (Straight) to override default driving behavior.

---

## 🔵 Phase 4: Human-Robot Interaction
**Objective:** Control the vehicle using advanced computer vision techniques.

### 4.1 hand_control.py (Gesture Control)
Use Google MediaPipe to recognize hand gestures as control inputs.
*   **Key Concept:** Skeleton tracking and finger counting logic.
*   **Logic:** Show 1-2 fingers to start the engine; show a full palm (5 fingers) to stop.

---

## 🏁 Phase 5: The Full Self-Driving (FSD) System
**Objective:** Orchestrate all nodes into a unified autonomous stack.

The ultimate goal is running the **`fsd_master.launch.py`**, which launches the coordinated versions of these nodes (`fsd_lane_keep`, `fsd_lidar_safety`, `fsd_mission_manager`).
*   **Safety Priority:** LiDAR Safety > YOLO Maneuvers > Lane Keeping.
*   **Execution:**
    ```bash
    ros2 launch mycar fsd_master.launch.py
    ```
