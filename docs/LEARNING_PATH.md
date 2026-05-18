# MentorPi M1 robot Mastering Path: Full Self-Driving (FSD) Engineering

Welcome to the MentorPi M1 robot FSD engineering track. This specialized learning path discards general robotics concepts to focus strictly on building a **Full Self-Driving (FSD) Autonomous Vehicle**. You will learn to integrate Ackermann steering, computer vision for lane keeping, and AI for traffic recognition.

---

## 🟢 Step 1: Control Wheels
**Objective:** The "Hello World" of autonomous vehicles—establishing communication with the hardware and getting the drive motors to turn.

### 1.1 Hardware Architecture & Boot
*   **Main Controller:** STM32 handles real-time throttle and steering servo control.
*   **High-Level Compute:** Raspberry Pi / Jetson Nano running ROS 2.
*   **Initialization:** Master the `bringup.launch.py` sequence to bring the vehicle online and establish serial communication.

### 1.2 Drive-by-Wire & Kinematics
*   **Configuration:** Ensure the `MACHINE_TYPE` environment variable is set to `MentorPi_Acker` (Car-like steering) before launching.
*   **Motor Control:** Understand how `src/driver/controller/controller/odom_publisher_node.py` converts `/cmd_vel` (Twist) messages into specific steering angles for the front servo (`SetPWMServoState`) and throttle speeds for rear motors (`MotorsState`).

### 1.3 Actionable ROS 2 Steps
1.  **Start the base system (Drivers & Sensors):**
    ```bash
    ros2 launch bringup bringup.launch.py
    ```
2.  **Test Manual Control (Keyboard):**
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
    *Use **W/X** for forward/backward and **A/D** for steering.*
    *Alternatively, run the example script to publish test velocities directly:*
    ```bash
    ros2 run example body_control
    ```
3.  **Control via Command Line (Topic):**
    Publish velocity commands directly to the `/cmd_vel` topic for manual testing.
    ```bash
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
    ```
    *Adjust `linear.x` for speed (m/s) and `angular.z` for steering angle (rad).*

```mermaid
graph LR
    Input["Teleop / Example Script"] -- "/cmd_vel (Twist)" --> Controller["driver Node (Kinematics)"]
    Controller -- "Steering Angle" --> Servo["Front Servo"]
    Controller -- "Throttle" --> Motors["Rear Drive Motors"]
```

### 1.4 drive_node (Basic Movement)
Learn to create a formal ROS 2 package and your first control node.

1.  **Create the Package:**
    Open a terminal and run these commands:
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python mycar --dependencies rclpy geometry_msgs
    ```

2.  **Write the Control Node:**
    Create `src/mycar/mycar/drive_node.py`:
    ```python
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    class DriveNode(Node):
        def __init__(self):
            super().__init__('drive_node')
            self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
            self.timer = self.create_timer(0.5, self.timer_callback)

        def timer_callback(self):
            msg = Twist()
            msg.linear.x = 0.2
            msg.angular.z = 0.1
            self.publisher_.publish(msg)
            self.get_logger().info('My car is driving...')

    def main(args=None):
        rclpy.init(args=args)
        node = DriveNode()
        rclpy.spin(node)
        rclpy.shutdown()
    ```

3.  **Register and Run:**
    Add `'drive_node = mycar.drive_node:main'` to `setup.py`, then build:
    ```bash
    cd ~/ros2_ws && colcon build --packages-select mycar
    source ~/.zshrc
    ros2 run mycar drive_node
    ```

### 1.5 square_move (Sequence Logic)
Build on your package by adding a node that follows a timed sequence to drive in a square.

1.  **Write the Node:**
    Create `src/mycar/mycar/square_move.py`:
    ```python
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    import time

    class SquareMove(Node):
        def __init__(self):
            super().__init__('square_move')
            self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
            self.run_square()

        def run_square(self):
            self.get_logger().info('Starting Square Movement...')
            for i in range(4):
                self.get_logger().info(f'Driving Side {i+1}...')
                self.move(0.2, 0.0, 2.0)  # Forward 2s
                self.get_logger().info(f'Turning Right...')
                self.move(0.0, -0.5, 1.5) # Turn Right 1.5s
            self.move(0.0, 0.0, 1.0)      # Stop
            self.destroy_node()
            rclpy.shutdown()

        def move(self, linear_x, angular_z, duration):
            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            end_time = time.time() + duration
            while time.time() < end_time:
                self.publisher_.publish(msg)
                time.sleep(0.1)

    def main(args=None):
        rclpy.init(args=args)
        node = SquareMove()
    ```

2.  **Register and Run:**
    Add `'square_node = mycar.square_move:main'` to `setup.py`, then build:
    ```bash
    cd ~/ros2_ws && colcon build --packages-select mycar
    source ~/.zshrc
    ros2 run mycar square_node
    ```

### 1.6 color_control (Vision Interaction)
Learn to control the robot using real-time color detection from the camera.

1.  **Write the Node:**
    Create `src/mycar/mycar/color_control.py` (Refer to the source code for the optimized Mean Color version).
    *   **Green Detected:** Starts continuous movement.
    *   **Red Detected:** Stops the robot.

2.  **Register and Run:**
    Add `'color_control_node = mycar.color_control:main'` to `setup.py`, then build:
    ```bash
    cd ~/ros2_ws && colcon build --packages-select mycar
    source ~/.zshrc
    ros2 run mycar color_control_node
    ```

---

## 🟡 Step 2: Lane Keep Assist (LKA)
**Objective:** Equip the vehicle with "eyes" to understand roads and lane markings, ensuring it stays centered on the track.

### 2.1 Environmental Scanning (Camera Setup)
*   Interface with the front-facing camera via [[peripherals]] (`usb_cam` or `depth_camera`).
*   Understand image topics and basic image filtering.

### 2.2 OpenCV Lane Detection Basics
To follow a lane, the robot needs to perform these steps in real-time:

1.  **Color Space Conversion:** Convert BGR to **HSV** or **LAB** to make it easier to isolate specific colors (like yellow/white lines) regardless of lighting.
2.  **Region of Interest (ROI):** Crop the image to look only at the floor. We don't need to process the ceiling or walls.
3.  **Thresholding:** Create a binary mask where the lane is white and everything else is black.
4.  **Centroid Calculation:** Find the horizontal center of the white area.

### 2.3 DIY: Create "lane_detect_node"
Write a node that finds the center of a line and visualizes it.

1.  **Write the Node:**
    Create `src/mycar/mycar/lane_detect_node.py`:
    ```python
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
            self.subscription = self.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image', self.image_callback, 10)
            self.bridge = CvBridge()
            
            # Threshold for Yellow/White Line (Adjust as needed)
            self.lower_line = np.array([20, 100, 100])
            self.upper_line = np.array([50, 255, 255])

        def image_callback(self, msg):
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, _ = cv_image.shape
            
            # 1. Focus on the bottom half (ROI)
            roi = cv_image[int(h/2):h, 0:w]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            # 2. Find the line
            mask = cv2.inRange(hsv, self.lower_line, self.upper_line)
            
            # 3. Find Moments (to calculate center)
            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                
                # Draw the center point
                cv2.circle(roi, (cx, cy), 5, (0, 0, 255), -1)
                
                # 4. Simple Steering Logic
                error = cx - w/2
                twist = Twist()
                twist.linear.x = 0.1
                twist.angular.z = -float(error) / 100 # Proportional control
                self.publisher_.publish(twist)
                
            cv2.imshow("Lane Detection", roi)
            cv2.waitKey(1)

    def main(args=None):
        rclpy.init(args=args)
        node = LaneDetectNode()
        rclpy.spin(node)
    ```

2.  **Register and Run:**
    Add `'lane_detect_node = mycar.lane_detect_node:main'` to `setup.py`, then build:
    ```bash
    cd ~/ros2_ws && colcon build --packages-select mycar
    source ~/.zshrc
    ros2 run mycar lane_detect_node
    ```

---

## 🟠 Step 3: Traffic Sign Navigation
**Objective:** Integrate AI object detection to allow the vehicle to make routing decisions based on physical traffic signs.

### 3.1 YOLOv5 Traffic Sign Detection
*   **The Code:** Run the `yolo_detect.py` node from the [[yolov5_ros2]] package.
*   **Model:** Ensure you load a `.pt` model trained to recognize traffic signs (e.g., Left Turn, Right Turn, Stop).
*   **Data Output:** The node publishes to `/object_detect` containing the class name, bounding box, and confidence score.

### 3.2 Decision Logic (State Machine)
*   **Integration:** You will need to write a custom control node (e.g., in the `example` package) that subscribes to both the LKA output and the YOLOv5 output.
*   **Execution:** 
    *   *Default State:* Relay the lane-keeping `/cmd_vel` to the motor drivers.
    *   *Event Trigger:* If YOLO detects a "Turn Left" sign with >80% confidence, temporarily override the LKA command with a hard left turn command (`angular.z`) for a set duration before resuming lane keeping.

### 3.3 Actionable ROS 2 Steps
1.  **Launch YOLOv5 Detection:**
    ```bash
    ros2 launch yolov5_ros2 yolov5_ros2.launch.py
    ```
2.  **View Real-time Detections:**
    ```bash
    ros2 topic echo /yolov5_ros2/object_detect
    ```
3.  **Develop the Override Node:** Create a new Python script in `src/example/example/` that listens to `interfaces/msg/ObjectsInfo` and `geometry_msgs/Twist`, and controls the vehicle accordingly.

```mermaid
graph TD
    Cam["Camera"] --> YOLO["YOLOv5 Inference"]
    YOLO -- "Detected: Turn Right" --> Logic["Decision Node"]
    LKA["Lane Keep Assist"] -- "Default Steering" --> Logic
    Logic -- "Override LKA" --> Move["Turn Wheels Right"]
```

---

## 🔴 Step 4: Autonomous Parking
**Objective:** The ultimate FSD challenge—recognize a specific parking zone and execute a complex maneuver to park the vehicle safely.

### 4.1 Target Recognition
*   Use YOLOv5 or OpenCV template matching to specifically search for the "P" (Parking) sign.
*   Once detected, calculate the distance to the sign using the bounding box size or depth camera data.

### 4.2 Parking Maneuver Sequence
*   **State 1: Approach.** Slow down the vehicle's forward velocity (`linear.x`) as the bounding box of the "P" sign grows larger.
*   **State 2: Alignment.** Use Ackermann kinematics to steer into the parking spot.
*   **State 3: Stop & Secure.** Once the vehicle is inside the spot (distance threshold reached), publish a zero velocity command to brake. You can also publish to `/ros_robot_controller/set_buzzer` to signal parking is complete.

### 4.3 Actionable ROS 2 Steps
1.  **Launch the required perception nodes** (Camera, YOLO, LKA).
2.  **Run your Capstone Node:** Execute your custom script that acts as the state machine:
    ```bash
    ros2 run example your_fsd_capstone_node
    ```
    *This custom script will serve as the "Velocity Multiplexer" shown in the architecture graph below.*

---

## 📈 FSD Development Roadmap (Goal Progression)
This graph illustrates the milestone-driven progression to achieve Full Self-Driving capabilities in this project.

```mermaid
flowchart TD
    %% Milestone 1
    M1((Step 1:<br>Control Wheels)):::milestone
    M1 --> T1[Ackermann Kinematics]
    M1 --> T2[Serial Control via /cmd_vel]
    T1 --> M2
    T2 --> M2

    %% Milestone 2
    M2((Step 2:<br>Lane Keep Assist)):::milestone
    M2 --> T3[OpenCV Line Tracking]
    M2 --> T4[Color Thresholding]
    M2 --> T5[Centroid Steering Logic]
    T3 --> M3
    T4 --> M3
    T5 --> M3

    %% Milestone 3
    M3((Step 3:<br>Traffic Sign Navigation)):::milestone
    M3 --> T6[YOLOv5 Sign Detection]
    M3 --> T7[Steering Override Logic]
    T6 --> M4
    T7 --> M4

    %% Milestone 4
    M4((Step 4:<br>Autonomous Parking)):::goal
    M4 --> T8[Park Sign Recognition]
    M4 --> T9[Distance Estimation]
    M4 --> T10[Parking Maneuver Sequence]
    T8 --> Goal(((Full Self-Driving<br>Achieved))):::goal
    T9 --> Goal
    T10 --> Goal

    classDef milestone fill:#2f74c0,stroke:#333,stroke-width:2px,color:#fff;
    classDef goal fill:#288b45,stroke:#333,stroke-width:4px,color:#fff;
```

---

## 🧩 FSD System Architecture (ROS 2 Node Graph)
Below is the detailed node and topic architecture that powers the FSD features of the MentorPi M1 robot.

```mermaid
graph TD
    %% Hardware Layer
    subgraph "Hardware Drivers & Kinematics"
        STM32["ros_robot_controller_node"]
        Odom["odom_publisher_node (Ackermann)"]
        Camera["usb_cam / depth_camera Node"]
        Lidar["lidar_node (MS200/LD19)"]
    end

    %% Perception Layer
    subgraph "Perception (ADAS)"
        LaserFilter["laser_filters Node"]
        LKA["line_following.py (Lane Keeping)"]
        YOLO["yolo_detect.py (YOLOv5)"]
    end
    
    %% Decision Logic
    subgraph "FSD Decision Logic"
        TrafficNav["Traffic Sign Navigation Node"]
        ParkLogic["Autonomous Parking Node"]
    end

    %% Connections - Perception
    Lidar -- "/scan_raw" --> LaserFilter
    LaserFilter -- "/scan" --> LKA
    Camera -- "/image_raw" --> LKA
    Camera -- "/image_raw" --> YOLO
    
    %% Connections - Kinematics
    STM32 -- "Encoders/IMU" --> Odom
    
    %% Connections - Control Override & Commands
    LKA -- "/cmd_vel_lane" --> Multiplexer{"Velocity Multiplexer"}
    YOLO -- "/object_detect (Signs)" --> TrafficNav
    YOLO -- "/object_detect ('P' Sign)" --> ParkLogic
    TrafficNav -- "/cmd_vel_override" --> Multiplexer
    ParkLogic -- "/cmd_vel_park" --> Multiplexer
    
    Multiplexer -- "/cmd_vel (Final)" --> Odom
    Odom -- "MotorsState / SetPWMServoState" --> STM32
    
    %% Styling
    classDef hardware fill:#607d8b,stroke:#333,stroke-width:2px,color:#fff;
    classDef perception fill:#c94d2c,stroke:#333,stroke-width:2px,color:#fff;
    classDef logic fill:#6c429c,stroke:#333,stroke-width:2px,color:#fff;
    
    class STM32,Odom,Camera,Lidar hardware;
    class LaserFilter,LKA,YOLO perception;
    class Multiplexer,TrafficNav,ParkLogic logic;
```