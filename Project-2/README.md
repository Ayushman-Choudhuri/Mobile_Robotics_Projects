
# Project 2: Go Chase It!

## Project Overview

In this project two ROS packages have been created inside the `catkin_wrc\src`. 
They are `drive_bot` and the `ball_chaser` whcih will be used in Gazebo. The steps to design the robot, hourse it in the world and program it ro chase a white ball is mentioned below: 

`drive_bot`

 * Create a `my_robot` ROS package to hold the robot, the white ball and the world. 
 * Design a differential drive robot with the Unified Robot Description Format. Add two sensors to your robot: a lidar and a camera. Add Gazebo plugins for your robot’s differential drive, lidar, and camera. 
 * House the robot inside the world which was built in the **Build My World** Project.
 * Add a white-coloredball to the gazebo world and savea new copy of this in the world. 
 * the `world.launch` file should launch your world with the white colored ball and your robot.

 `ball_chaser`

 * Create a `ball_chaser` ROS package to hold your C++ nodes. 
 * Write a `drive_bot` C++ node that will provide a `ball_chaser/command_robot` service to drive the robot by controlling its linear x and angular z velocities. The service should publish to the wheel joints and return back the requested velocities. 
 * Write a `process_image` C++ node that reads your robot's camera image, analyzes it to determine the presence and position of a white ball. If a white ball exists in theimage, your node should request a service via a client to drive the robot towards it. 
 * The `ball_chaser.launch` should run both the `drive_bot` and the `process_image` nodes.

 
## Dependencies

Please make sure the following dependencies are met before running the project: 

* Gazebo >= 7.0
* ROS Kinetic
* make >= 4.1(mac,linux) , 3.81 (Windows)
    - **Linux**: make is installed by default 
    - **Mac**: Install [Xcode](https://developer.apple.com/xcode/features/)
    - **Windows**: [Installation Instructions](https://gnuwin32.sourceforge.net/packages/make.htm)
    
* gcc/gcc++ >= 5.4
    - **Linux**: gcc/gcc++ is installed by default
    - **Mac**: Install [X Code](https://developer.apple.com/xcode/features/)
    - **Windows**: Install [MinGW](https://sourceforge.net/projects/mingw/)




## Setup

* **Step 1**: Make sure the dependencies are met
* **Step 2**: Open bash/terminal and clone the project repository
* **Step 3**: Execute the following line in the command line to update info and upgrade the outdated packages and dependencies on your system.
  ```bash
  sudo apt-get update && sudo apt-get upgrade -y
  ```
* **Step 4**: Build and run the project
## Directory Structure

``` bash
.Project-2
├── ball_chaser                         # ball_chaser package
│   ├── CMakeLists.txt                  # compiler instructions
│   ├── launch                          # launch folder for the launch files
│   │   └── ball_chaser.launch          
│   ├── package.xml                     # package info
│   ├── src                             # source folder for all the C++ scripts
│   │   ├── drive_bot.cpp   
│   │   └── process_image.cpp
│   └── srv                             # service folder for ROS services
│       └── DriveToTarget.srv
└── my_robot
    ├── CMakeLists.txt                  # Compiler instructions
    ├── launch                          # launch folder for the launch files
    │   ├── robot_description.launch
    │   └── world.launch
    ├── meshes                          # meshes folder for hokuyo sensor
    │   └── hokuyo.dae
    ├── package.xml                     # package info
    ├── urdf                            # urdf folder for xacro files
    │   ├── my_robot.gazebo
    │   └── my_robot.xacro
    └── worlds                          # world files
        ├── empty.world
        └── office.world



```
## Running the Project

* **Step 1** : Clone the repository
  ```bash
  git clone https://github.com/Ayushman-Choudhuri/Udacity-Robotics-Software-Engineer-Nanodegree
  ```
* **Step 2** : add the `ball_chaser` and `my_robot` to your catkin workspace 

* **Step 3**: Open your catkin_workspace and make
  ```bash
  cd /home/workspace/catkin_ws/

  catkin_make
  ```

* **Step 4**: Launch my_robot in Gazebo (both world and plugins): 
    
  ```bash
  source devel/setup.bash 

  roslaunch my_robot world.launch
  ```

* **Step 5**:  launch `ball_chaser` and `process_image` nodes
  ```bash
  cd /home/workspace/catkin_ws/

  source devel/setup.bash

  roslaunch ball_chaser ball_chaser.launch
  ```

* **Step 6**: Visualize node connections 
  ```bash
  cd /home/workspace/catkin_ws/

  source devel/setup.bash

  rosrun rqt_image_view rqt_image_view
  ```
