> # ⚠️ This Branch is for RSBP5
>  
> This `jazzy-rsbp5` branch is intended for RSBP5.  
> For Control PC Jazzy version, please switch to the `jazzy` branch.

# 📌 Project Overview

This repository implements an **OpenManipulator-X Pick-and-Place** system based on ArUco marker–based eye-in-hand calibration, using the MoveIt 2 API and ROS 2 action-based robot control.

The system converts ArUco marker coordinates detected from a camera into the robot base frame using a calibrated hand-eye transformation, refines the target pose, and executes a Pick & Place scenario.

The core design goal is to **decouple perception from robot execution**, while enabling flexible task sequencing through a state-machine-based control flow and ROS 2 Action-based robot execution.

## 🎥 Demo Video
[![Pick & Place Demo](https://img.youtube.com/vi/JyPYKrjtD1M/maxresdefault.jpg)](https://youtube.com/shorts/M9DBwQ9MoKU)

> Click the image to watch the full demo on YouTube.
[https://youtu.be/JyPYKrjtD1M](https://youtube.com/shorts/M9DBwQ9MoKU)
> 

## ✅ Pick & Place Workflow

> 1. Detect ArUco markers from the camera stream
> 2. Extract marker poses and transform them into the robot base frame
> 3. Refine target poses through tracking and filtering
> 4. Publish refined object poses to the Pick & Place node
> 5. Trigger the Pick & Place task
> 6. Execute the sequence: open gripper → pick → grip → lift → place → release → return
> 7. Repeat the cycle while auto-run is enabled and valid objects are detected



## ✅ Pick & Place System Function
> * Camera-to-base pose transformation
> * Multi-object tracking with duplicate ID handling
> * Pose refinement through buffering, filtering, and stability checks
> * ROS 2 Action-based task execution
> * FSM-based Pick & Place execution
> * Trigger-based continuous auto-run control

## ✅ Implementation Environment

> OS: Ubuntu 24.04 LTS
>
> ROS 2: Jazzy
>
> Language: Python + Cpp
>
> Motion Planning: MoveIt 2
>
> Manipulator : OpenManipulator-X
>
> Camera : Logitech C270

## 🧩 Execution Instructions (RSBP5)

### Quick Start Guide
1. Install `ros-jazzy-desktop` and `ros-dev-tools` on your rsbp5
2. Set the same ROS domain ID in `.bashrc` : `export ROS_DOMAIN_ID=XX`
3. Set USB Port Permissions ```sudo usermod -aG dialout $USER```
4. Clone the Repository (jazzy-rsbp5 branch)
5. Install ROS2 Dependencies (run in the workspace root)
```
sudo rosdep init 
rosdep update
rosdep install --from-paths src -y --ignore-src
```
6. Check and align the versions of OpenCV (4.13.0.92), NumPy (1.26.4), SciPy (1.11.4), transforms3d (0.4.1), and Flask (3.1.2)
7. Build the Workspace ```colcon build```
8. Source the Workspace : ```source ~/ros2_ws/install/setup.bash```
9. Create and apply udev rules ```ros2 run open_manipulator_bringup om_create_udev_rules```
10. Check the current USB latency with following command (should be 1) ```cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer```
11. Configure udev rules to assign persistent device names to the USB camera (/dev/camera_c270) and the OpenManipulator port (/dev/ttyDYNAMIXEL).

### Terminal Execution
```
ros2 launch open_manipulator_bringup rsbp_full_bringup.launch.py # Launch the full bringup pipeline
ros2 launch openmanipulator_task_executor keyboard_trigger_node.py # Trigger for Pick & Place execution
```

## 🧠 System Architecture

### 1️⃣ robot_interface.cpp

(C++ | Action Server)

Wraps the MoveIt 2 MoveGroupInterface API into a dedicated ROS 2 node by exposing only the motion and gripper functions required for the Pick & Place task.

Implements these functions as ROS 2 Action Servers so that higher-level task nodes can send sequential motion requests and execute continuous Pick & Place behaviors in a structured way.

Separates low-level robot execution from task-level logic.

### 2️⃣ robot_interface_client.py

(Python | Non-Node Action Client Helper)

Provides a Python helper class for accessing the robot_interface action server from higher-level task nodes.

This class is not a ROS node itself.
Instead, it is imported and used inside the main Pick & Place node, which owns the actual ROS node context.

Its purpose is to let Python-based task logic interact with the C++ motion backend in a simple and reusable way, while keeping action, service, and state-handling code separate from the main task flow.

Although these methods could have been implemented directly inside the main Pick & Place node, they were separated into this file to improve readability, modularity, and maintainability.

Main Methods

* send_move_to_pose() — sends a Cartesian pose goal

* send_move_to_named() — sends a named target goal

* send_move_to_joint_pose() — sends a joint-space goal

* send_gripper() — sends a gripper command

* send_emergency_stop() — sends an emergency stop command

* joint_callback() — requests FK from current joint states

* fk_response_callback() — updates cached end-effector pose

Acts as a bridge between the Python task node and the C++ robot control backend, improving readability and maintainability.


### 3️⃣ coordinate_extractor.py

(Python | Perception, Object Tracking, and Pose Refinement Node)

Detects ArUco markers from the camera stream, converts their poses into the robot base frame using the hand-eye calibration transform, refines the coordinates for stable robot execution, and publishes the final target poses to the Pick & Place node.

This node was designed not just to detect markers, but to generate robot-usable target poses from noisy visual input through tracking, filtering, and pose refinement.

**Main Responsibilities**

1. Detects multiple ArUco markers from the live camera stream
2. Estimates marker poses with solvePnP
3. Transforms camera-frame poses into the robot base frame using hand-eye calibration
4. Refines target poses through **multi-frame buffering, Z-score–based outlier removal, stability checks, and task-oriented orientation generation for reliable Pick & Place execution**.
5. Publishes refined object poses through /aruco/refined_objects

**Key Design Points**
* Supports multiple-object tracking, including separate handling of objects that share the same ArUco ID

* Buffers object poses over multiple frames for temporal tracking (up to 60 frames per object, with tracked objects cleared after 20 consecutive missed frames)

* (targets are rejected if the filtered positional standard deviation is greater than 0.005 m on x, y, or z)

* Generates grasp-oriented target orientation for robot execution instead of using raw marker orientation directly

Converts raw ArUco detections into stable and robot-executable target poses for Pick & Place.


### 4️⃣ open_manipulator_x_pickandplace.py

(Python | Main Pick & Place Task Node)

Implements the main Pick & Place task logic as a ROS 2 node.

This node receives refined target poses from the **coordinate extractor**, sends motion and gripper requests through **robot_interface_client**, and executes the full Pick & Place sequence using a finite-state-machine-based control flow.

**Main Responsibilities**

1. Receives refined object poses from /aruco/refined_objects
2. Starts task execution from /pick_and_place/start
3. Executes the Pick & Place sequence through staged FSM logic
4. Sends motion and gripper commands via robot_interface_client
5. Applies position offsets before executing the pick motion
6. Publishes completion signals through /pick_and_place/done
7. Supports repeated execution through auto-run mode

**Pick & Place Sequence**
'idle - pick_open - pick - grip - lift - place - release - return - done - idle'

Acts as the main task orchestrator that connects refined perception output to sequential robot execution.


### 5️⃣ keyboard_trigger_node.py

(Python | Execution Trigger Node)

Publishes a trigger message to start or stop the main Pick & Place task node.

When turned on, it enables continuous Pick & Place execution as long as valid objects are detected.
When turned off, it disables task execution so that Pick & Place does not run even if objects are detected.


### 🧩 Key Features

✔ ROS 2 Action-based task execution

✔ Python–C++ bridge structure for task-level robot control

✔ MoveIt MoveGroupInterface–based robot control

✔ FSM-based Pick & Place task execution with auto-run control

✔ ArUco marker–based hand-eye calibration

✔ Multi-object tracking, including separate handling of duplicate ArUco IDs

✔ Multi-frame pose refinement with outlier rejection and stability checks


### 🎯 Design Philosophy

Separation of concerns
(Perception / Pose Refinement / Motion Backend / Task Logic / Execution Trigger)

Action-based robot execution
for structured sequential motions and reusable task control

Robot-usable perception output
by refining noisy visual detections into stable Pick & Place target poses

Modular Python–C++ integration
for readability, maintainability, and easier system extension

Practical system design
focused on repeatable real-robot execution rather than one-shot demo scripts
