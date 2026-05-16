# System Overview - Calibration Package

The `calibration` package provides essential tools to ensure the MentorPi M1 robot robot's sensors and movement parameters are accurate.

## Core Calibration Tools

### 1. Movement Calibration
Ensures that when you command the robot to move 1 meter or turn 90 degrees, it does so precisely.
- **`calibrate_linear.py`**:
    - **Method:** Robot moves forward for a set time at a set velocity.
    - **User Action:** Measure the actual distance moved and update the `linear_correction_factor` in the driver config.
- **`calibrate_angular.py`**:
    - **Method:** Robot rotates 360 degrees (or a set number of turns).
    - **User Action:** Measure the actual rotation and update the `angular_correction_factor`.

### 2. Camera Calibration
Corrects lens distortion for the USB or Depth cameras.
- **Tools:** Often uses standard ROS 2 `camera_calibration` package with a checkerboard.
- **Output:** Saves results to `camera_info.yaml` (typically in the `peripherals` or `yolov5_ros2` config folders).

### 3. Servo Calibration
Sets the "zero" position for PWM and Bus servos.
- **Logic:** Adjusts the offset value so that a command of `1500` (neutral) corresponds to the mechanical center of the servo.

## Calibration Workflow
1.  **Preparation:** Place the robot in a clear, flat area.
2.  **Execution:** Run the specific calibration node (e.g., `ros2 run calibration calibrate_linear`).
3.  **Measurement:** Record the physical result.
4.  **Update:** Edit the YAML configuration files in the `driver/controller/config` directory.
5.  **Verification:** Run the test again to confirm accuracy.

## Why Calibrate?
Without calibration, the robot's odometry will "drift," causing SLAM maps to become distorted and navigation to fail as the robot loses track of its true position.
