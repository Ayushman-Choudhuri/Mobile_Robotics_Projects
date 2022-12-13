
# Project-4: Map my World 

## Project Overview

This project aims to create a 2D occupancy grid and 3D octomap from a simulated environment using a custom robot with the RTAB-Map package. 

RTAB-Map(Real-Time Appearence Based Mapping) is a popular solution for SLAM to develop robots that can map environments in 3D. RTAB-Map has a good speed and 
memory management, and it provides  custom developed tools for information analysis. Most importantly, the quality of the documentation on [ROS Wiki](http://wiki.ros.org/rtabmap_ros) is very high. Being able to leverage RTAB-Map with your own robots will lead to a solid foundation for mapping and localization well beyond this Nanodegree program.
For this project we will be using the rtabmap_ros package, which is a ROS wrapper (API) for interacting with RTAB-Map. Keep this in mind when looking at the relative documentation.

This project aims to achieve the following: 

* Development of a package to interface with the rtabmap_ros package
* Build upon the robot localization project to make the necessary changes to interface the robot with the RTAB-Map. An example of this is the addition of an RGB-D camera. 
* Ensure that all the files are in appropriate place and all links are connected, naming is properly setup and topics are correctly mapped. Furthermore generate the appropriate launch files to launch the robot and map its surroundings. 
* After the robot is launched, teleop around the room to generate a proper map of the environment. 


## Dependencies

* Gazebo >= 7.0  
* ROS Kinetic  
* ROS navigation package  
```
sudo apt-get install ros-kinetic-navigation
```
* ROS map_server package  
```
sudo apt-get install ros-kinetic-map-server
```
* ROS move_base package  
```
sudo apt-get install ros-kinetic-move-base
```
* ROS amcl package  
```
sudo apt-get install ros-kinetic-amcl
```
* ROS rtabmap-ros package  
```
sudo apt-get install ros-kinetic-rtabmap-ros
``` 

* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Directory Structure

``` bash 

.Project-4                                      # Map My World Project
├── ball_chaser                                 # ball_chaser package
│   ├── CMakeLists.txt                          # compiler instructions
│   ├── launch                                  # launch folder for launch files
│   │   └── ball_chaser.launch
│   ├── package.xml                             # package information
│   ├── src                                     # source folder for c++ scripts
│   │   ├── drive_bot.cpp
│   │   └── process_image.cpp
│   └── srv                                     # service folder for ROS services
│       └── DriveToTarget.srv
├── my_robot                                    # my_robot_package
│   ├── CMakeLists.txt                          # compiler instructions
│   ├── config                                  # config folder for configuration files
│   │   ├── base_local_planner_params.yaml
│   │   ├── costmap_common_params.yaml
│   │   ├── global_costmap_params.yaml
│   │   ├── local_costmap_params.yaml
│   │   └── __MACOSX
│   ├── launch                                  # launch folder for launch files
│   │   ├── amcl.launch
│   │   ├── localization.launch
│   │   ├── mapping.launch
│   │   ├── readme
│   │   ├── robot_description.launch
│   │   ├── teleop.launch
│   │   └── world.launch
│   ├── maps                                    # folder for map files
│   │   ├── map.pgm
│   │   └── map.yaml
│   ├── meshes                                  # folder for mesh files
│   │   └── hokuyo.dae
│   ├── package.xml                             # package information
│   ├── urdf                                    # urdf folder for xacro files 
│   │   ├── my_robot.gazebo
│   │   └── my_robot.xacro
│   └── worlds                                  # world files
│       ├── empty.world
│       └── office.world
├── pgm_map_creator                             # pgm_map_creator cloned repo
└── teleop_twist_keyboard                       # teleop_twist_keyboard cloned repo
└── Images and Videos                           # Images and videos of the project
    ├── Gazebo World.png
    ├── Loop Closures.png
    ├── Screen Recoding.mp4
    └── SLAM Map.png


```
## Running the Project

* **Step 1**: Clone the repository
  ```bash
  git clone https://github.com/Ayushman-Choudhuri/Udacity-Robotics-Software-Engineer-Nanodegree
  ```
* **Step 2**: add the packages to your catkin workspace 

* **Step 3**: Open your catkin_workspace and make
  ```bash
  cd /home/workspace/catkin_ws/
  catkin_make
  ```

* **Step 3**:Launch my_robot in Gazebo to load both the world and plugins  
```
roslaunch my_robot world.launch
```  
* **Step 4**:Launch teleop_twist_keyboard node, open a new terminal, enter  
```
cd /home/workspace/RoboND-Term1-P4-Map-My-World/catkin_ws/
source devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```  
* **Step 5**:Launch teleop_twist_keyboard node, open a new terminal, enter  
```
cd /home/workspace/RoboND-Term1-P4-Map-My-World/catkin_ws/
source devel/setup.bash
roslaunch my_robot mapping.launch
```  
* Testing  
Send move command via teleop package to control your robot and observe real-time visualization in the environment `rtabmapviz`.  
rtabmap-databaseViewer ~/.ros/rtabmap.db

* View database
Once you statisfied with your move, press `Ctrl + c` to exit then view your database with
```
rtabmap-databaseViewer ~/.ros/rtabmap.db
```
Remember to rename your `~/.ros/rtabmap.db` before your next attempt since it will be deleted due to the launch file setting in `mapping.launch`

## Suggestions

1. It's recommended to update and upgrade your environment before running the code.  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
2. Remember to rename your `~/.ros/rtabmap.db` before your next attempt since it will be deleted due to the launch file setting in `mapping.launch`
