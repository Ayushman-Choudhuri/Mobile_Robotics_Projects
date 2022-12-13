
# Home Service Robot 
<p align="center">
<img align="center" src="https://github.com/Ayushman-Choudhuri/Udacity-Robotics-Software-Engineer-Nanodegree/blob/main/Home-Service-Robot/Images%20and%20Videos/video.gif">
</p>
## Project Overview 

This project combines all the principles of mapping , localization and navigation

The project aims to achieve the following goals: 

### Mapping

* Create a `test_slam.sh` script file and launch it to manually test test_slam
* A functional map of the environment should be created which would be used for localization and navigation tasks. 

### Localization and Navigation 

* Create a `test_navigation.sh` script file to launch it for manual navigation test 
* Robot should be able to navigate in the environment after a 2D Nav goal command is issued. 
* Create a `pick-objects.sh` file that will send multiple goals for the robot to reach. 
* The robot must travel to the desired pickup zone, display a message andthen once it reaches the destination , it should wait for 5 seconds and then travel to the desired drop off zone. After this a message should be displayed that it has reached the drop off zone

### Home Service Functions 

Create a `add_marker.sh` file that will publish a marker to rviz.The marker should initially be published at the pickup zone. After 5 seconds it should be hidden. Then after another 5 seconds it should appear at the drop off zone.
Also writea `home_service.sh` file that will run all the nodes in this project. The home service robot should be simulated as follows: 

* Initially show the marker at the pickup zone 
* Hide the marker once the robot reaches the pickup zone 
* Wait for 5 seconds to simulate a pickup 
* Show the marker at the drop off zone once your robot reaches it. 


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
.Home-Service-Robot
├── add_markers                             # add_marker package
│   ├── CMakeLists.txt                      # Compiler Instructions
│   ├── launch                              # Folder for launch files
│   │   └── view_navigation.launch
│   ├── package.xml                         # package information
│   └── src                                 # Folder for C++ scripts
│       └── add_markers.cpp
├── CMakeLists.txt                          # compiler instructions
├── pick_objects                            # pick_objects package
│   ├── CMakeLists.txt                      # compiler Instructions
│   ├── package.xml                         # package information
│   └── src                                 # Folder for C++ scripts
│       └── pick_objects.cpp
├── README.md                               # Project README
├── rvizConfig                              # rvizConfig Package
│   └── home_service_rvizConfig.rviz
├── scripts                                 # Shell script files
│   ├── add_marker.sh                       # Shell script to model virtual objects
│   ├── home_service.sh                     # Shell script to launch home service robot demo
│   ├── launch.sh                           
│   ├── pick_objects.sh                     # Shell script to send multiple goals
│   ├── test_navigation.sh                  # Shell script to test localization and test_navigation
│   └── test_slam.sh                        # Shell script to test SLAM
├── slam_gmapping                           # gmapping_demo.launch file
├── turtlebot                               
├── turtlebot_interactions                  # view_navigation.launch file
└── turtlebot_simulator                     #turtlebot_world.launch file package


```
## Setup

* Step 1: Meet the Dependencies
* Step 2: Clone repository 
* Step 3: On a terminal execute the following command

```bash
sudo apt-get update && sudo apt-get upgrade -y
```
* Step 4: clone the following packages into the src folder of your project

``` bash
git clone https://github.com/ros-perception/slam_gmapping.git  
git clone https://github.com/turtlebot/turtlebot.git  
git clone https://github.com/turtlebot/turtlebot_interactions.git  
git clone https://github.com/turtlebot/turtlebot_simulator.git  
```

* Step 5: Build and run your code


## Running the Project

* **Step 1** : Clone the repository
  ```bash
  git clone https://github.com/Ayushman-Choudhuri/Udacity-Robotics-Software-Engineer-Nanodegree
  ```
* **Step 2** : Add the packages to the src folder of your workspace

* **Step 3** : In the src folder clone the necessary repositories

```bash
git clone https://github.com/ros-perception/slam_gmapping.git  
git clone https://github.com/turtlebot/turtlebot.git  
git clone https://github.com/turtlebot/turtlebot_interactions.git  
git clone https://github.com/turtlebot/turtlebot_simulator.git
```

* **Step 3**: Open your catkin_workspace and make
  ```bash
  cd /home/workspace/catkin_ws/
  catkin_make
  source devel/setup.bash
  ```

* **Step 3**: Launch the home service robot

```bash 
./src/scripts/home_service.sh
```

## Suggestions
