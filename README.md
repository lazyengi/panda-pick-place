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
You should already have ROS Noetic installed. Follow also the getting started guides for franka_ros and realsense-ros. 

You should have installed MoveIt 1, follow the [getting start guide](https://moveit.github.io/moveit_tutorials/doc/getting_started/getting_started.html) otherwise. If you want to have my same MoveIt configuration, clone the repo of my panda_moveit_config.

Follow the instructions on the github page of the the realsense-ros to install the nodes of the camera.

Clone or download panda_pick_place, panda_vision_model and panda_pick_place_msgs into \<your catkin workspace\>/src/ folder and build it.

If you want to train your own model for the computer vision, my tips is to install yolo by following this [guide](https://docs.ultralytics.com/it/quickstart/#conda-docker-image).

# Project stucture
By launching the panda_pick_place.launch file you are going to run three nodes:

1. The vision node (panda_vision)
2. The robot controller node (panda_pick_place)
3. The control hub (panda_controller)

The computer vision node provides the topics that serve the vision scope. It reads the image from the Intel RealSense node and processes the image data to determine the object's spatial position.

All the topics about the robot control are from the panda_pick_place node.

Eventually the control hub provides a guid to easily interact with the robot and also synchronize the other nodes. 

# Available Configurations
Under the config folder, there is the pick_place.yaml file. In this file you can customize:
- handshaking_freq: the frequency of the handshaking between the controller, the pick_place node and the vision node;
- default_pick_pose: you can choose a default pick pose that you can then change at runtime by the action provided in the control hub. You can choose only one pick pose. All the poses of this file are in the joint space, thus an array of 7;
- default_place_poses: you can set a list of place poses which index should correspond to the index of the class of the object detection. You can change these too from the control hub;
- place_poses_count: the length of the default_place_poses list;
- default_velocity_scaling_factor: is the valocity scaling factor of the arm. It's raccomanded not to go over 0.7;
- default_acceleration_scaling_factor: is the acceleration scaling factor of the arm. It's raccomanded not to go over 0.7;
- gripper_closed_epsilon: when the gripper closes around a thin object each finger has it's position and the sum of the two position is the width of the object grasped. So this param (which unit is meters) says that under <gripper_closed_epsilon> meters the gripper is considered closed without grasping an object. Can happen that when the gripper is closed, the width between fingers still result around 0.0005 meters;
- rotation_step: when the arm chooses which object to grasp should know the position in the space but also the orientation. From the object detection you have only a box around the object with sides parallels to the image frame. With the fingers of the gripper in the configuration to grasp small object, you can grasp objects wide only 8 centimeters. So to choose the best angle, the arm makes multiple detection maximising or minimising the width of the bounding box. After each detection the hand of the arm rotates by <rotation_step>;
- model/path: is the path of the object detection model. You can change it at runtime from the control hub;
- vision/depth_epsilon: the Intel RealSense d435i camera has a infrared depth camera. So you can have some configuration of the objects on the table that creates some holes in the pointcloud. When the model carry out the detection I take the center of the bounding box with the highest confidence. Not to fail right away the detection I try to find a depth point around the center of the bounding box. If the pointcloud has a hole, the depth is at 0 meters. You can go on searching for a valid depth point until some depth over the <depth_epsilon>;

// TODO image of the explanation

- vision/depth_step: is the count of pixel to move from the previous point;
- tf/rate_freq: the frequency of the broadcast of the transform matrix from the panda_hand to the camera_link frame by the tf2 broadcaster;
- tf/panda_ee_link: the name of the end effector link fram of the robot;
- tf/camera_link: the name of the camera frame;
- tf/tf_matrix: the omogeneous transform matrix from the <panda_ee_link> to the <camera_link>;
- gui/width: the width of the gui windows;
- gui/heigth: the heigth of the gui windows;
- guid/columns_count: the count of the columns of the gui

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