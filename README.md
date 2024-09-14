![logo università di Firenze](https://github.com/lazyengi/panda-pick-place/blob/master/assets/images/logo_unifi_500.jpg)
# Introduction

<!-- TODO: Mettici la gif della demo-->

This project is part of the thesis work for the bachelor's degree program in Electronic and Telecommunications Engineering, Automation track, at the University of Florence.
The goal is to set up the Panda robot from Franka Emika for pick and place operation. The idea is to realize a project as modular and open to update as possible.
The project is splitted in two main parts:
1. Robot control
2. Object detection
### 1. Robot control
Learn to control the robot and carry out simple operation like pick and place. 
### 2. Object detection
Learn to create a neural network that can detect simple object that should be picked and placed.

Finally combine the two functionality to solve the initial problem
# Requirements
The robot model is [Panda from Franka Emika](https://frankaemika.github.io/docs/overview.html). The camera model is [Intel RealSense d435i](https://dev.intelrealsense.com/docs/sdk-knowledge-base).
This project is developed with [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) on Ubuntu 20.04. It depends on the following packages:
- [franka_ros](https://frankaemika.github.io/docs/franka_ros.html)
- [panda_moveit_config](https://github.com/lazyengi/panda_moveit_config)
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy)
- [panda_vision_model](https://github.com/lazyengi/panda_vision_model)
- [panda_pick_place_msgs](https://github.com/lazyengi/panda_pick_place_msgs)

# Installation
You should already have ROS Noetic installed. Also, follow the getting started guides for franka_ros and realsense-ros.

You should have MoveIt 1 installed; otherwise, follow the [getting start guide](https://moveit.github.io/moveit_tutorials/doc/getting_started/getting_started.html). If you want to use my MoveIt configuration, clone the repository of my `panda_moveit_config`.

Follow the instructions on the GitHub page of realsense-ros to install the camera nodes.

Clone or download `panda_pick_place`, `panda_vision_model`, and `panda_pick_place_msgs` into \<your catkin workspace\>/src/ folder and build them.

If you want to train your own model for computer vision, my tip is to install YOLO by following this [guide](https://docs.ultralytics.com/it/quickstart/#conda-docker-image).

# Project stucture
By launching the panda_pick_place.launch file, you will run three nodes:

1. The vision node (`panda_vision`)
2. The robot controller node (`panda_pick_place`)
3. The control hub (`panda_controller`)

The computer vision node provides the topics that serve the vision scope. It reads the image from the Intel RealSense node and processes the image data to determine the object's spatial position.

All the topics about the robot control are from the `panda_pick_place` node.

Eventually the control hub provides a guid to easily interact with the robot and also synchronize the other nodes. 

# Available Configurations
Under the config folder, there is the pick_place.yaml file. In this file you can customize:
- **handshaking_freq**: the frequency of the handshaking between the controller, the pick_place node and the vision node;
- default_pick_pose: you can choose a default pick pose that you can then change at runtime by the action provided in the control hub. You can choose only one pick pose. All the poses of this file are in the joint space, thus an array of 7;
- **default_place_poses**: you can set a list of place poses which index should correspond to the index of the class of the object detection. You can change these too from the control hub;
- **place_poses_count**: the length of the default_place_poses list;
- **default_velocity_scaling_factor**: is the valocity scaling factor of the arm. It's raccomanded not to go over 0.7;
- **default_acceleration_scaling_factor**: is the acceleration scaling factor of the arm. It's raccomanded not to go over 0.7;
- **gripper_closed_epsilon**: when the gripper closes around a thin object each finger has it's position and the sum of the two position is the width of the object grasped. So this param (which unit is meters) says that under <gripper_closed_epsilon> meters the gripper is considered closed without grasping an object. Can happen that when the gripper is closed, the width between fingers still result around 0.0005 meters;
- **rotation_step**: when the arm chooses which object to grasp should know the position in the space but also the orientation. From the object detection you have only a box around the object with sides parallels to the image frame. With the fingers of the gripper in the configuration to grasp small object, you can grasp objects wide only 8 centimeters. So to choose the best angle, the arm makes multiple detection maximising or minimising the width of the bounding box. After each detection the hand of the arm rotates by <rotation_step>;
- **model/path**: is the path of the object detection model. You can change it at runtime from the control hub;
- **vision/depth_epsilon**: the Intel RealSense d435i camera has a infrared depth camera. So you can have some configuration of the objects on the table that creates some holes in the pointcloud. When the model carry out the detection I take the center of the bounding box with the highest confidence. Not to fail right away the detection I try to find a depth point around the center of the bounding box. If the pointcloud has a hole, the depth is at 0 meters. You can go on searching for a valid depth point until some depth over the <depth_epsilon>;

// TODO image of the explanation

- **vision/depth_step**: is the count of pixel to move from the previous point;
- **tf/rate_freq**: the frequency of the broadcast of the transform matrix from the panda_hand to the camera_link frame by the tf2 broadcaster;
- **tf/panda_ee_link**: the name of the end effector link fram of the robot;
- **tf/camera_link**: the name of the camera frame;
- **tf/tf_matrix**: the omogeneous transform matrix from the <panda_ee_link> to the <camera_link>;
- **gui/width**: the width of the gui windows;
- **gui/heigth**: the heigth of the gui windows;
- **guid/columns_count**: the count of the columns of the gui

# Run the project
You have to run the robot interface node from panda_moveit_config package, by running:

```
roslaunch panda_moveit_config franka_control.launch  robot_ip:=<robot_ip> load_gripper:=true robot:=panda
```

make sure to insert your robot ip address.

Then you have to run the camera node with the color camera aligned to the depth camera:

```
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

Now you can start the project by running:

```
roslaunch panda_pick_place panda_pick_place.launch
```

// TODO image of the control hub 

# Control hub
Listed in the first row there are the nodes we need to control. Under the name of the node you have the status of the node. You can start the control when both nodes are ready. At the end of the row there is the stop button that sends the shutdown message to all the nodes. If you need to restart the nodes (i.e. the franka_control node have had an issue) you can tap on the 'Reset Nodes' button. If you want to go step by step during the pick and place (i.e. stop after the detection and ask for the command to go for the pick), turn the debug on.

Then you have the row where you che change the model you want to use for the object detection.

Eventually there are all the possible actions. Here the details:
- *Home*: homing of the arm;
- *Open Gripper*: self explainatory;
- *Close Gripper*: self explainatory;
- *Set Pick Pose*: before tapping on this button move the arm at the pick position you want to have. 

Here's how you can do it:

1. Without shutting any nodes, go on the browser where you have the franka interface and set the *Programming operation*;

// TODO Franka interface image

2. Light clicking both buttons on the gripper you can move the arm in the pose you want;

3. Back on the franka interface switch in *Execution operation*;

4. Now you tap on the *Set pick pose* button on the control hub. If some error is thrown tap *Reset nodes* and try again with set pick pose button.

Back to the actions on the hub:

- *Set Place Pose*: you can set as many poses as specified in the config file;
- *Start Detection*: make a single detection;
- *Pick and Place*: make a single pick and place operation;
- *Go to Pick Pose*: make the arm go to the pick pose;
- *Go to Place Pose*: make the arm go to the place pose;
- *Clean Table*: start the routine of pick placing object until there's nothing to grasp any more.

# Roadmap
I will explain some of the problem that can be evalueated.

1. The debug is not working properly. When reached a breakpoint the operator node should print a message on the debug topic (i.e. '/panda_pick_place/debug') and wait for the go message. There is some bug or may be rethinked the algorithm.

2. At start of pick pose routine, the arm is in the pick pose where make first detection. When it chooses object to grasp, it lowers a bit over it and starts the process to optimize picking angle. If there are close object with high detectability the computer vision can detect another object different from the first choice. Is there a better way to do it?

3. During my research the arm controller plans the trajectory from a pose to another optimizing some parameters. Thus can happen to see it choose a long and dangerous trajectory that can also lead to reach joint velocity limits. You can plan the trajectory using some waypoints or place virtual obstacols to force the trajectory?

4. Object distorsion leads to arm collision with hard objects

# My project
I choose to train the object detection model to recognize plastic bottle or cups and cans. So the goal of this experiment was to throw on a table some mixed object of these two classes and then put each object in the correct box.

I leave you with the routine that closes the project.

// TODO VIdeo dell'esperimento


Special thanks goes to professor Benedetto Allotta for giving me the opportunity to play with the arm, Mirco Vangi for patiently listen and giving me a big hand every day in the lab, Nicola Secciani for the big and precious advice and all the Unifi Robotics team.

*Giuseppe A. Dimola, University of Florece.*