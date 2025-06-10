# IR_2425
## GROUP 19


Member1: Caduceo Andrea, andrea.caduceo@studenti.unipd.it

Member2: Girardello Sofia, sofia.girardello.1@studenti.unipd.it

Member3: Gizzarone Manuel, manuel.gizzarone@studenti.unipd.it

This repository contains two ROS-based projects developed in Python and using the Gazebo simulation environment. Both projects involve the Tiago robot performing two different tasks: object localization in the environment using AprilTags, and pick-and-place operations with objects of different shapes.

# Assignment 1

This folder contains the code for the object localization project. Specifically, it focuses on detecting AprilTags, transforming their poses into the map frame, and coordinating with a navigation system. The project includes an action server that listens for target IDs and manages AprilTag detections using `tf` transformations.

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


# Assignment 2

This folder contains the code for the pick-and-place project. In particular, the picking operation involves localizing objects on a pick-table through the use of AprilTags. Subsequently, the obtained poses are transformed into the robot's reference frame using `tf`. This data then serves as the basis for generating suitable pick poses specific to each object type, which are subsequently employed for the planning and execution of robot arm movements via `MoveIt`.
For the placing operation on a specific place-table, the desired placement positions are generated using information from the external service `/straight_line_srv`. These positions are specifically constrained to lie along a straight line, with the line's parameters randomly generated on request by the service. The resulting poses are then transformed into the robot's frame using `tf` and subsequently used for executing movements via `MoveIt`.
  
## Features

-  **Pick Action Server**: Listens for a specific object's pose, computes a suitable pick-pose for each type of object, and then performs the corresponding movement.

-  **Place Action Server**: Listens for a specific placement pose, computes a suitable place-pose for each type of object, and then performs the corresponding movement.

-  **Pose Transformation**: Transforms AprilTag poses from the camera frame to the robot frame using `tf`.

  
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
   roslaunch assignment_2 tiago_full_simulation.launch

2. Start the place_routine node (Node A):
   ```bash
   rosrun assignment_2 place_routine.py 

3. Start the object_detection_node node (Node B):
   ```bash
   rosrun assignment_2 object_detection_node.py

4. Start the pick_object_server node (Node C):
   ```bash
   rosrun assignment_2 pick_object_server.py
