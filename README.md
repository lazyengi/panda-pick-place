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
You should already have ROS Noetic installed. Follow also the getting started guides for franka_ros and realsense-ros. Clone or download panda_pick_place, panda_vision_model and panda_pick_place_msgs into \<your catkin workspace\>/src/ folder and build it.
