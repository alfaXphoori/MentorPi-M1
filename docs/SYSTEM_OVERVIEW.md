# MentorPi M1 robot Workspace: System Overview

This page is a one-page summary of the MentorPi M1 software stack. The whole system is built on **ROS 2**, where hardware drivers, perception, autonomy, and AI modules communicate through standard topics, services, and actions.

## System at a Glance

| Layer | Purpose | Main Docs |
| --- | --- | --- |
| Hardware | Connect to motors, servos, IMU, LiDAR, and cameras | [[driver]], [[peripherals]] |
| Bringup & Apps | Start the robot and run ready-made behaviors | [[bringup]], [[app]], [[example]] |
| Spatial Intelligence | Build maps and move autonomously | [[slam]], [[navigation]] |
| AI & Perception | Detect objects, signs, and reason about scenes | [[yolov5_ros2]], [[large_models]] |
| Interfaces & Coordination | Share common messages and multi-robot logic | [[interfaces]], [[large_models_msgs]], [[multi]] |
| Simulation | Test the full stack without hardware risk | [[simulations]] |

## Core Standards

- **Middleware:** ROS 2 topics, services, and actions
- **Chassis Types:** Mecanum, Ackermann, and Tank
- **Development Style:** Modular packages that can run alone or as part of a larger launch system

## Typical Data Flow

`Sensors -> Drivers -> Perception / AI -> Decision Logic -> /cmd_vel -> Robot Base`

## Start Here

- **Boot the robot:** [[bringup]]
- **Fix hardware drift:** [[calibration]]
- **Follow the step-by-step learning path:** [[LEARNING_PATH]]
