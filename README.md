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

OS: Ubuntu 22.04 LTS (Jammy)

ROS 2: Humble Hawksbill

Language: Python + Cpp

Vision: OpenCV, ArUco

Motion Planning: MoveIt 2

Hardware: Manipulator, Logitech C270

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
