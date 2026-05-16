# MentorPi M1 robot Workspace: System Overview

This document provides a technical bird's-eye view of the MentorPi M1 robot robot's software ecosystem, built entirely on **ROS 2**.

## Architecture Layers

### 1. Hardware Abstraction ([[driver]] & [[peripherals]])
The lowest layer that talks to the physical hardware.
- **Drivers:** Handle serial communication with the STM32 board to control motors, servos, and read IMU data.
- **Sensors:** Provide drivers for 2D LiDAR and 3D depth cameras, including data filtering.

### 2. Base Behaviors ([[app]] & [[example]])
Pre-configured applications that combine sensors and actuators.
- Includes line following, object tracking, hand gesture control, and AR visualization.
- Examples serve as a template for custom development.

### 3. Spatial Intelligence ([[slam]] & [[navigation]])
The layer that allows the robot to understand its environment.
- **SLAM:** Building maps using LiDAR and Camera data.
- **Navigation:** Autonomous path planning and obstacle avoidance using the Nav2 stack.

### 4. Advanced AI ([[yolov5_ros2]] & [[large_models]])
The "Human-Like" intelligence layer.
- **YOLO:** Real-time object detection (people, signs, objects).
- **LLM/VLM:** Understanding voice commands and reasoning about visual scenes using Large Language Models.

---

## Technical Standards

### Communication (ROS 2)
- **Middleware:** Uses standard ROS 2 topics, services, and actions.
- **Custom Interfaces:** Defined in [[interfaces]] and [[large_models_msgs]] to ensure consistent data structures.

### Chassis Types Supported
- **Mecanum:** 4-wheel omnidirectional base for tight space maneuvers.
- **Ackermann:** 4-wheel car-like steering for natural driving dynamics.
- **Tank:** High-traction differential drive.

### Coordination ([[multi]])
Supports multi-agent systems where multiple robots share maps and coordinate movements in formation.

### Simulation ([[simulations]])
A high-fidelity Gazebo environment to test all layers without risking physical hardware.

---

## Quick Navigation
*   **Ready to start?** Check the [[bringup]] package.
*   **Need to fix a drift?** See the [[calibration]] guide.
*   **Want to learn?** Follow the [[LEARNING_PATH]].
