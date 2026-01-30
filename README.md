> **⚠️ Default branch: ROS 2 Humble**
>  
> This `main` branch is based on **ROS 2 Humble Hawksbill**.  
> For ROS 2 Jazzy, please switch to the `jazzy` branch.

# 📌 Project Overview

This repository implements an OpenManipulator-X Pick & Place system based on
ArUco Marker–based Hand-Eye Calibration and ROS 2 Action-based robot control.

The system converts ArUco marker coordinates detected from a camera into the robot base frame using a calibrated hand-eye transformation, refines the target pose, and executes a Pick & Place scenario through a modular and extensible architecture.

The core design goal is to decouple perception, coordinate processing, and robot execution, while enabling flexible task sequencing via ROS 2 Actions.

## 🎥 Demo Video

[![Pick & Place Demo](https://img.youtube.com/vi/JyPYKrjtD1M/maxresdefault.jpg)](https://www.youtube.com/watch?v=JyPYKrjtD1M)

> Click the image to watch the full demo on YouTube.

https://youtu.be/JyPYKrjtD1M

## ✅ Implementation Environment

OS: Ubuntu 22.04 LTS / Ubuntu 24.04 LTS

ROS 2: Humble / Jazzy

Language: Python + Cpp

Vision: OpenCV, ArUco Marker

Motion Planning: MoveIt 2

Manipulator model : OpenManipulator-X

Camera : Logitech C270



## 🧩 Execution Instructions (RSBP & Control PC)

## Prerequisite

1. Set the same ROS domain ID to `.bashrc' : ```export ROS_DOMAIN_ID=XX```

### Humble

* Since an actual RSBP4 device was not available, this setup has not been physically verified.
However, it is expected to work correctly by running the bringup on the RSBP4 and the control-related launch files on the control PC.

##### RSBP4
1. Install ros-humble-ros-base and ros-dev-tools on the RSBP4 and set up the OpenManipulator packages by following the official guide:https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/#setup
2. Keep only the bringup and description packages, remove the rest, and build the workspace with 'colcon build --symlink-install' (only the hardware launch file from bringup is required).

   
##### Control PC
1. Install ros-humble-desktop and ros-dev-tools on the control pc and set up the OpenManipulator packages by following the official guide:https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/#setup
2. When you have reached ' <img width="316" height="46" alt="image" src="https://github.com/user-attachments/assets/29aa078b-a6bd-4cdd-bc6d-d2c5116a4cd9" /> ' , instead of cloning ' <img width="885" height="103" alt="image" src="https://github.com/user-attachments/assets/62e0a280-8644-4827-89df-12ed8edb883e" /> ' , clone humble branch of this repository
3. Check the self.cap = cv2.VideoCapture('/dev/camera_c270') line in open_manipulator_x_pickandplace.py and update it to match your camera device.
4. build the workspace with 'colcon build --symlink-install'


Terminal Execution
```
# RSBP4
ros2 launch open_manipulator_x_bringup hardware.launch.py

# Control PC
ros2 launch open_manipulator_x_moveit_config move_group.launch.py
ros2 launch openmanipulator_task_executor pickandplace_bringup.launch.py
ros2 launch openmanipulator_task_executor keyboard_trigger_node.py
```
### Jazzy

##### RSBP4
1. Install ros-jazzy-ros-base and ros-dev-tools on the RSBP4 and set up the OpenManipulator packages by following the official guide:https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/#setup
2. Keep only the bringup and description packages, remove the rest, and build the workspace with 'colcon build --symlink-install' (only the hardware launch file from bringup is required).

   
##### Control PC
1. Install ros-jazzy-desktop and ros-dev-tools on the control pc and set up the OpenManipulator packages by following the official guide:https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/#setup
2. When you have reached ' <img width="319" height="40" alt="image" src="https://github.com/user-attachments/assets/3a5c93e2-c371-4442-bdf5-7c97a9d797f1" />
 ' , instead of cloning ' <img width="872" height="105" alt="image" src="https://github.com/user-attachments/assets/bdf4b4c9-b80a-4d19-8b98-2f12ec8de09a" />
 ' , clone jazzy branch of this repository
3. Check the self.cap = cv2.VideoCapture('/dev/camera_c270') line in open_manipulator_x_pickandplace.py and update it to match your camera device.
4. build the workspace with 'colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'

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

(Python | Action Client Wrapper)

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
