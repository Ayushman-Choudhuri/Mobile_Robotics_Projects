# Udacity-Robotics-Software-Engineer-Nanodegree

This repository contains all the projects I have worked on during the nano degree

## Project 1: Build My World

Tasks: 
   1. Build a single floor wall structure using the Building Editor tool in Gazebo. Apply at least one feature, one color, and optionally one texture to your structure. Make sure there's enough space between the walls for a robot to navigate.
   2. Model any object of your choice using the Model Editor tool in Gazebo. Your model links should be connected with joints.
   3. Import your structure and two instances of your model inside an empty Gazebo World.
   4. Import at least one model from the Gazebo online library and implement it in your existing Gazebo world.
   5. Write a C++ World Plugin to interact with your world. Your code should display “Welcome to ’s World!” message as soon as you launch the Gazebo world file.

## Project 2: Go Chase it!

Tasks: 

In this project, you should create two ROS packages inside your catkin_ws/src: the drive_bot and the ball_chaser. Here are the steps to design the robot, house it inside your world, and program it to chase white-colored balls:

drive_bot:
    
1. Create a my_robot ROS package to hold your robot, the white ball, and the world.
2. Design a differential drive robot with the Unified Robot Description Format. Add two sensors to your robot: a lidar and a camera. Add Gazebo plugins for your robot’s differential drive, lidar, and camera. The robot you design should be significantly different from the one presented in the project lesson. 
3. Minimum required changes are adjusting the color, wheel radius, and chassis dimensions.
4. Create a new world, which is different from the world you built in the Build My World project and house your robot inside that world.
5. Add a white-colored ball to your Gazebo world and save a new copy of this world.
6. The world.launch file should launch your world with the white-colored ball and your robot.

ball_chaser:
    
1. Create a ball_chaser ROS package to hold your C++ nodes.
2. Write a drive_botC++ node that will provide a ball_chaser/command_robot service to drive the robot by controlling its linear x and angular z velocities. 3. The service should publish to the wheel joints and return back the requested velocities.
4. Write a process_image C++ node that reads your robot’s camera image, analyzes it to determine the presence and position of a white ball. If a white ball exists in the image, your node should request a service via a client to drive the robot towards it.
5. The ball_chaser.launch should run both the drive_bot and the process_image nodes.


## Project 3: Where am I ?

Tasks: 

1. Create a ROS package that launches a custom robot model in a custom Gazebo world
2. Utilize the ROS AMCL package and the Tele-Operation / Navigation Stack to localize the robot
3. Explore, add, and tune specific parameters corresponding to each package to achieve the best possible localization results


## Project 4: Map my World (Ongoing)

Tasks: 

1. Develop a package to interface with the rtabmap_ros package.
2. Build on top of the localization project to make the necessary changes to interface the robot with RTAB-Map. An example of this is the addition of an RGB-D camera.
3. Ensure that all files are in the appropriate places, all links are properly connected, naming is properly setup and topics are correctly mapped. Furthermore you will need to generate the appropriate launch files to launch the robot and map its surrounding environment.
4. Teleop around the room to generate a proper map of the environment.
