# 🤖 SortiBot – Robotic Waste Sorting Simulation

## Overview

SortiBot is a ROS 2 and Gazebo–based robotic simulation designed for automated waste sorting using vision-guided manipulation. The system consists of a 4-DOF robotic arm mounted on a platform next to a conveyor belt, equipped with an eye-in-hand camera for object detection and classification. The simulation supports segregation of objects into metal, plastic, and paper/wood categories.

This repository provides the full simulation stack, including the Gazebo world, robot description (URDF), camera integration, and scaffolding for perception and control development.

---

## ✨ Features

✅ Gazebo world with:
- Conveyor belt  
- Sorting bins (metal, plastic, paper)  
- Robot mounting platform  

✅ Custom 4-DOF robotic arm (URDF)  
✅ Wrist-mounted RGB camera (Gazebo ROS camera plugin)  
✅ Camera image published to ROS 2 topics  
✅ Modular structure for perception + manipulation  
✅ Ready for object detection integration (TFLite / Edge Impulse)

---

## 🖥️ System Requirements

- Ubuntu 22.04  
- ROS 2 Humble  
- Gazebo (Classic)  
- colcon  
- Python 3.10+

---

## 📁 Repository Structure

```text
sortibot_ws/
├── src/
│   ├── sortibot_arm/
│   │   ├── urdf/
│   │   │   └── sortibot_4dof.urdf
│   │   ├── launch/
│   │   │   └── spawn_arm.launch.py
│   ├── sortibot_gazebo/
│   │   ├── worlds/
│   │   │   └── sortibot_world.world

📥 Clone the Repository

git clone https://github.com/Mansi-1120/sortibot-simulation.git
cd sortibot-simulation


---

🛠️ Build the Workspace

cd ~/sortibot_ws
colcon build
source install/setup.bash


---

🚀 Launch the Simulation

Start Gazebo World

gazebo sortibot_world.world

Spawn the Robotic Arm

ros2 launch sortibot_arm spawn_arm.launch.py


---

📷 Camera Topics

Verify camera topics:

ros2 topic list | grep camera

Expected topic:

/sortibot/camera/image_raw

View camera output:

ros2 run rqt_image_view rqt_image_view


---

🧱 Dummy Objects

Dummy objects (metal, plastic, paper) are included in the Gazebo world to simulate detection and sorting scenarios.

These objects support perception pipeline testing before deploying real datasets.


---

📊 Dataset Usage

Pre-collected datasets annotated using Label Studio will be used for training object detection models (TensorFlow Lite / Edge Impulse).

Simulation objects are used only for integration testing, not training.


---

🔭 Future Work

Object detection inference (TFLite / Edge Impulse)

Pick-and-place pipeline

Arm motion planning

ROS 2 perception node

Sorting automation logic



---

👤 Maintainer

Mansi Singh
Robotics | AI | Manipulation
