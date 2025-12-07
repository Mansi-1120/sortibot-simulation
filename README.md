# 🤖 SortiBot – Robotic Waste Sorting Simulation

## Overview

SortiBot is a *ROS 2 + Gazebo–based robotic simulation* designed for automated waste sorting using *vision-guided manipulation*.  
The system consists of a *custom 4-DOF robotic arm* mounted next to a conveyor belt and equipped with an *eye-in-hand RGB camera* for real-time object perception.

The simulation supports segregation of objects into the following categories:

- *metal*
- *plastic*
- *paper / wood*

This repository provides the *complete simulation and perception stack*, enabling integration between vision and manipulation modules for end-to-end waste sorting automation.



## ✨ Features

- ✅ Gazebo world with:
  - Conveyor belt
  - Sorting bins (metal, plastic, paper/wood)
  - Robot mounting platform  
- ✅ Custom-designed 4-DOF robotic arm (URDF)
- ✅ Wrist-mounted RGB camera using Gazebo ROS camera plugin
- ✅ ROS 2 image streaming to /sortibot/sortibot_camera/image_raw
- ✅ TensorFlow Lite–based *real-time perception node*
- ✅ High-level control commands published to /sortibot/control
- ✅ Modular structure for perception and manipulation separation

---

## 🖥 System Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo (Classic)
- colcon
- Python 3.10+
- TensorFlow (CPU – TensorFlow Lite runtime)

---

## 📂 Repository Structure

```text
sortibot_ws/
├── src/
│   ├── sortibot_arm/              # 4-DOF arm, URDF and spawn launch
│   │   ├── urdf/
│   │   │   └── sortibot_4dof.urdf
│   │   ├── launch/
│   │   │   └── spawn_arm.launch.py
│
│   ├── sortibot_gazebo/           # Gazebo world and environment
│   │   ├── worlds/
│   │   │   └── sortibot_world.world
│   │   ├── launch/
│   │   │   └── sortibot_world.launch.py
│
│   ├── sortibot_perception/       # Vision-based perception node
│   │   ├── src/
│   │   │   └── perception_node.py
│   │   ├── models/
│   │   │   ├── model.tflite
│   │   │   └── labels.txt
│   │   └── launch/
│   │       └── perception.launch.py



🏗 Create the Workspace and Clone the Repository

# 1. Create a ROS 2 workspace
mkdir -p ~/sortibot_ws
cd ~/sortibot_ws

# 2. Clone the repository
git clone https://github.com/Mansi-1120/sortibot-simulation.git .




🔨 Build the Workspace

cd ~/sortibot_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

📌 Make sure to source install/setup.bash in every new terminal.

🚀 Launch the Simulation

1️⃣ Start the Gazebo World

source /opt/ros/humble/setup.bash
source ~/sortibot_ws/install/setup.bash
ros2 launch sortibot_gazebo sortibot_world.launch.py

This launches:

Conveyor belt

Sorting bins

Robot platform

Dummy objects for testing



---

2️⃣ Spawn the Robotic Arm

source /opt/ros/humble/setup.bash
source ~/sortibot_ws/install/setup.bash
ros2 launch sortibot_arm spawn_arm.launch.py

You should now see:

1. The 4-DOF robotic arm


2. Mounted on the platform


3. Facing the conveyor and bins





📷 Camera Topics

ros2 topic list | grep camera

Expected topic:

/sortibot/sortibot_camera/image_raw

To visualize:

ros2 run rqt_image_view rqt_image_view




🧠 Perception Node (Implemented ✅)

The perception module performs real-time object classification using a TensorFlow Lite model exported from Edge Impulse.

Run Perception

ros2 launch sortibot_perception perception.launch.py

Published Topics

Topic	Type	Description

/sortibot/object_class	std_msgs/String	Detected class label (metal, plastic, paper_wood)
/sortibot/control	std_msgs/String	High-level control command for motion


Control Commands

pick_metal

pick_plastic

pick_paper_wood

idle


These commands are designed to be consumed by a motion control node to trigger corresponding pick-and-place actions.




🧱 Dummy Objects

The Gazebo world includes colored dummy objects representing:

Metal-like

Plastic-like

Paper/wood-like


These are used for:

Camera framing validation

End-to-end pipeline testing (camera → inference → control)





🗂 Dataset Usage

1. Real-world datasets are collected and annotated externally (COCO / Edge Impulse).


2. Models are trained using Edge Impulse or TensorFlow.


3. Exported .tflite models are integrated into this ROS 2 perception node.



Simulation objects are not used for training, only for pipeline validation.




🔭 Future Work

Motion planning with MoveIt 2

Closed-loop pick-and-place execution

Gripper control integration

Multi-object tracking on the conveyor

Optimized inference for embedded Edge AI




👤 Maintainer

Mansi Singh
Robotics · AI · Manipulation
