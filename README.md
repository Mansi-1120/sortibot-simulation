#🤖 SortiBot – Robotic Waste Sorting Simulation 

## Overview

SortiBot is a ROS 2 + Gazebo–based robotic simulation designed for automated waste sorting using vision-guided manipulation.  
The system consists of a 4-DOF robotic arm mounted on a platform next to a conveyor belt, equipped with an eye-in-hand camera for object detection and classification.  
The simulation supports segregation of objects into **metal**, **plastic**, and **paper/wood** categories.

This repository provides the full simulation stack, including:

- Gazebo world
- Robot description (URDF)
- Camera integration
- Scaffolding for perception and control development

---

## ✨ Features

- ✅ Gazebo world with:
  - Conveyor belt
  - Sorting bins (metal, plastic, paper)
  - Robot mounting platform  
- ✅ Custom 4-DOF robotic arm (URDF)
- ✅ Wrist-mounted RGB camera (Gazebo ROS camera plugin)
- ✅ Camera image published to ROS 2 topics
- ✅ Modular structure for perception + manipulation
- ✅ Ready for object detection integration (TensorFlow Lite / Edge Impulse)

---

## 🖥️ System Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo (Classic)
- `colcon`
- Python 3.10+

---

## 📂 Repository Structure

```text
sortibot_ws/              # ROS 2 workspace (local folder name can be anything)
├── src/
│   ├── sortibot_arm/     # 4-DOF arm + URDF + spawn launch
│   │   ├── urdf/
│   │   │   └── sortibot_4dof.urdf
│   │   ├── launch/
│   │   │   └── spawn_arm.launch.py
│   ├── sortibot_gazebo/  # Gazebo world + launch
│   │   ├── worlds/
│   │   │   └── sortibot_world.world
│   │   ├── launch/
│   │   │   └── sortibot_world.launch.py

🏗️ Create the Workspace and Clone the Repository

On any machine (including teammates), do:

# 1. Create a ROS 2 workspace
mkdir -p ~/sortibot_ws
cd ~/sortibot_ws

# 2. Clone this repository into the workspace root
git clone https://github.com/Mansi-1120/sortibot-simulation.git .

# Now your structure is:
# ~/sortibot_ws/src/sortibot_arm
# ~/sortibot_ws/src/sortibot_gazebo

🔨 Build the Workspace

#From inside the workspace:

cd ~/sortibot_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash


Make sure to source install/setup.bash in every new terminal before running ROS 2 commands.

🚀 Launch the Simulation
1. Start the Gazebo World
source /opt/ros/humble/setup.bash
source ~/sortibot_ws/install/setup.bash
ros2 launch sortibot_gazebo sortibot_world.launch.py


This will start Gazebo with:

- Conveyor belt

- Three colored bins (metal, plastic, paper)

- Arm platform

- Dummy objects on the conveyor

2. Spawn the Robotic Arm

#In a new terminal:

source /opt/ros/humble/setup.bash
source ~/sortibot_ws/install/setup.bash
ros2 launch sortibot_arm spawn_arm.launch.py


You should now see:

1. The 4-DOF robotic arm

2. Mounted on the gray platform

3. Facing the conveyor and bins

📷 Camera Topics

#Check available camera topics:

ros2 topic list | grep camera


#Expected topic:

/sortibot/camera/image_raw


#To view the camera output (if rqt_image_view is installed):

ros2 run rqt_image_view rqt_image_view


Then select /sortibot/camera/image_raw from the dropdown.

🧱 Dummy Objects

The Gazebo world includes simple colored dummy objects placed on the conveyor to simulate:

1. Metal-like objects

2. Plastic-like objects

3. Paper/wood-like objects

These objects are meant for:

1. Testing camera view and framing

2. Early integration testing of perception + motion

They are not used directly for training; they are only for simulation and pipeline testing.

🗂️ Dataset Usage

1. Real datasets (images of metal, plastic, paper/wood) are collected and annotated externally (e.g., Label Studio, COCO format).

2. These datasets will be used to train object detection or classification models (TensorFlow Lite / Edge Impulse).

Simulation objects are used for:

1.Verifying model integration

2. Testing the full loop: camera → prediction → motion

🔭 Future Work

1. Object detection inference node (TensorFlow Lite / Edge Impulse)

2. Pick-and-place pipeline

3. Arm motion planning

4. ROS 2 perception node

5. Automated sorting logic based on model outputs

👤 Maintainer

Mansi Singh
Robotics · AI · Manipulation
