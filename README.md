# IR_2425
# Assignment 1

# GROUP 19

Member1: Caduceo Andrea, andrea.caduceo@studenti.unipd.it

Member2: Girardello Sofia, sofia.girardello.1@studenti.unipd.it

Member3: Gizzarone Manuel, manuel.gizzarone@studenti.unipd.it

This repository contains code for a ROS-based project focused on detecting AprilTags, transforming their poses to the map frame, and coordinating with a navigation system. The project includes an action server that listens for target IDs and handles AprilTag detections using `tf` transformations.

## Features

- **Action Server**: Listens for target IDs, searches for AprilTags, and reports their positions.
- **Pose Transformation**: Transforms AprilTag poses from the camera frame to the map frame using `tf`.
- **Navigation Integration**: Notifies the navigation system when research begins and ends.

## Installation

1. Clone the repository into your ROS workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/SofiaGirardello/ir_2425_group_19.git
2. Build the workspace:
   ```bash
   cd ~/catkin_ws
   catkin_make
3. Source the workspace:
   ```bash
   source ~/catkin_ws/devel/setup.bash

## Running the program

1. Launch the entire system:
   ```bash
   roslaunch assignment_1 assignment_1.launch
2. Start the navigator node (choose if you want to use the CorridorNavigator or not):
   ```bash
   rosrun assignment_1 navigator.py _use_corridor_navigator:=False
3. Start the server node:
   ```bash
   rosrun assignment_1 server.py
4. Start the client node:
   ```bash
   rosrun assignment_1 client.py
