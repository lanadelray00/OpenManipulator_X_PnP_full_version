# 📌 Project Overview

This repository implements an **OpenManipulator-X Pick & Place system** based on
ArUco Marker–based Hand-Eye Calibration and ROS 2 Action-based robot control.

The system converts ArUco marker coordinates detected from a camera into the robot base frame using a calibrated hand-eye transformation, refines the target pose, and executes a Pick & Place scenario through a modular and extensible architecture.

The core design goal is to decouple perception and robot execution, while enabling flexible task sequencing via ROS 2 Actions.

## 🎥 Demo Video

[![Pick & Place Demo](https://img.youtube.com/vi/JyPYKrjtD1M/maxresdefault.jpg)](https://www.youtube.com/watch?v=JyPYKrjtD1M)

> Click the image to watch the full demo on YouTube.
https://youtu.be/JyPYKrjtD1M
> 
## ✅ Implementation Environment

OS: Ubuntu 24.04 LTS

ROS 2: Jazzy

Language: Python + Cpp

Motion Planning: MoveIt 2

Manipulator : OpenManipulator-X

Camera : Logitech C270



## 🧩 Execution Instructions (RSBP & Control PC)

## Prerequisite

1. Set the same ROS domain ID on both the **control PC** and the **RSBP5** by adding the following line to `.bashrc` -> `export ROS_DOMAIN_ID=XX`

### Jazzy

##### RSBP5
1. Install ros-jazzy-ros-desktop and ros-dev-tools on the RSBP5 and clone jazzy-rsbp5 branch
2. To enable communication with the hardware, add your user to the dialout group
```sudo usermod -aG dialout $USER```
3. Install ROS 2 Dependencies (run in the workspace root)
```
sudo rosdep init 
rosdep update
rosdep install --from-paths src -y --ignore-src
```
4. Build the Package ```colcon build```
5. Source the workspace ```source ~/ros2_ws/install/setup.bash```
6. Create and apply udev rules ```ros2 run open_manipulator_bringup om_create_udev_rules```
7. Check the current latency with following command (should be 1) ```cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer```
   
##### Control PC
1. Install ros-jazzy-desktop and ros-dev-tools on the control pc and set up the OpenManipulator packages by following the official guide:https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/#setup
2. When you have reached ' <img width="319" height="40" alt="image" src="https://github.com/user-attachments/assets/3a5c93e2-c371-4442-bdf5-7c97a9d797f1" />
 ' , instead of cloning ' <img width="872" height="105" alt="image" src="https://github.com/user-attachments/assets/bdf4b4c9-b80a-4d19-8b98-2f12ec8de09a" />
 ' , clone jazzy branch of this repository
3. build the workspace with ```colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release```

Terminal Execution
```
# RSBP5
ros2 launch open_manipulator_bringup open_manipulator_x.launch.py

# Control PC
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py
ros2 launch openmanipulator_task_executor pickandplace_bringup.launch.py
ros2 launch openmanipulator_task_executor keyboard_trigger_node.py
```

## 🧠 System Architecture

### 1️⃣ robot_interface.cpp

(C++ | Action Server)

Implements a ROS 2 Action Server using MoveGroupInterface

Provides a high-level robot control interface (IK solving, motion planning, execution)

Designed as a reusable backend for various task scenarios

Abstracts low-level MoveIt control from task logic

### 2️⃣ robot_interface_client.py

(Python | Action Client)

Python interface for accessing the C++ Action Server

Provides easy-to-use methods for motion requests

Enables Python-based task nodes to control the manipulator without direct MoveIt dependency

### 3️⃣ ArUco_coord_extractor.py

(Perception & Coordinate Processing)

Detects ArUco markers from the camera image

Applies Hand-Eye Calibration transformation matrix

Converts camera coordinates into the robot base frame

Outputs refined target poses independent of perception source
(Designed for easy replacement with YOLO, AprilTag, etc.)

### 4️⃣ open_manipulator_x_pickandplace.py

(Main Task Node)

Receives refined target coordinates

Executes the Pick & Place scenario using robot_interface_client

Encapsulates task logic separately from robot control and perception

Acts as the main orchestrator of the system

### 5️⃣ keyboard_trigger_node.py

(Trigger Node)

Publishes topic-based triggers to start task execution

Allows manual control of task flow (start / retry / sequencing)

Designed to be replaceable with external UI or higher-level planners

### 🧩 Key Features

✔ ROS 2 Action-based task execution

✔ MoveIt MoveGroupInterface–based robot control

✔ ArUco Marker–based Hand-Eye Calibration

✔ Camera → Base frame coordinate transformation

✔ Modular and extensible architecture

✔ Perception-independent coordinate interface

### 🎯 Design Philosophy

Separation of concerns
(Perception / Coordinate Processing / Motion Control / Task Logic)

Action-based execution for continuous and interruptible tasks

Future-proof structure for perception method replacement

Practical robot application focus, not demo-level scripts
