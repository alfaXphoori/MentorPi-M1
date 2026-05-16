# System Overview - Example Package

The `example` package provides standalone code snippets and simple nodes to demonstrate how to use individual features of the MentorPi M1 robot robot. It is the best starting point for developers.

## Categories of Examples

### 1. Basic Movement
- **`body_control`**: Simple commands to move the robot in different directions (Mecanum/Ackermann).
- **`self_driving`**: A basic implementation of obstacle-aware driving.

### 2. Vision Examples
- **`color_detect` & `color_track`**: How to find and follow a specific color using OpenCV.
- **`mediapipe_example`**: Using Google MediaPipe for face, hand, or pose detection.
- **`yolov5_detect`**: A minimal script to run a YOLOv5 model.
- **`qrcode`**: Scanning and decoding QR codes or Barcodes.

### 3. Hand Interaction
- **`hand_track`**: Keeping the camera centered on a hand.
- **`hand_trajectory`**: Recording the path a hand takes in front of the camera.
- **`hand_gesture_control`**: Mapping gestures (like a thumbs up) to robot actions.

### 4. Advanced Tasks
- **`color_sorting`**: Logic for identifying a color and moving it to a specific bin (using a gripper/arm).

## Learning Path
If you are new to the MentorPi M1 robot, follow this order:
1.  **Chassis Control:** Learn how to send `Twist` messages to move the robot.
2.  **Sensor Data:** Learn how to read `/scan` (LiDAR) and image topics.
3.  **Basic CV:** Run the `color_track` example.
4.  **Integration:** Combine vision with movement to create a complete behavior.

## How to Run
Most examples can be run using the standard ROS 2 command:
```bash
ros2 run example [example_name]
```
Check the source code in `example/example/` for specific topic names and parameters used in each demonstration.
