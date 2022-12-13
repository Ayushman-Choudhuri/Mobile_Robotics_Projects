
# Project-3: Where am I
![alt-text](https://github.com/Ayushman-Choudhuri/Udacity-Robotics-Software-Engineer-Nanodegree/blob/main/Project-3/Screenshots/Localization%202.png)
![alt-text](https://github.com/Ayushman-Choudhuri/Udacity-Robotics-Software-Engineer-Nanodegree/blob/main/Project-3/Screenshots/localization1.png)
## Project Overview 

This project utilizes the ROS AMCL package to accurately localize a movile robot inside a map in the Gazebo simulation environement. The project aims to achieve the following: 

* Create a ROS package that launches a custom robot model in a custom Gazebo world. 
* Utilize the ROS AMCL package and the Tele-Operation/Navigation stack to localize the robot. 
* Explore, add and tune specific parameters corresponding to reach package to achieve the best possible localization results. 




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
.Project-3
├── ball_chaser                             # ball_chaser package
│   ├── CMakeLists.txt                      # compiler instructions
│   ├── launch                              # folder for launch files
│   │   └── ball_chaser.launch
│   ├── package.xml                         # package information
│   ├── src                                 # Folder for C++ scripts
│   │   ├── drive_bot.cpp
│   │   └── process_image.cpp
│   └── srv                                 # Folder for ROS services
│       └── DriveToTarget.srv
├── my_robot                                # my_robot_package
│   ├── CMakeLists.txt                      # compiler instructions
│   ├── config                              # Folder for configuration files
│   │   ├── base_local_planner_params.yaml
│   │   ├── costmap_common_params.yaml
│   │   ├── global_costmap_params.yaml
│   │   ├── local_costmap_params.yaml
│   │   └── __MACOSX
│   ├── launch                              # Folder for launch files
│   │   ├── amcl.launch
│   │   ├── robot_description.launch
│   │   └── world.launch
│   ├── maps                                # Folder for maps
│   │   ├── map.pgm
│   │   └── map.yaml    
│   ├── meshes                              # Folder for hokuyo meshes
│   │   └── hokuyo.dae
│   ├── package.xml                         # Package information
│   ├── urdf                                # Folder for xacro files
│   │   ├── my_robot.gazebo
│   │   └── my_robot.xacro
│   └── worlds                              # Folder for world files 
│       ├── empty.world
│       └── office.world
├── pgm_map_creator                         # Create pgm map from Gazebo world file for ROS Localization                    
├── Screenshots                             # Folder for screenshots
    ├── localization1.png
    ├── Localization 2.png
    ├── Localization3.png
    └── Localization4.png


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

* **Step 5**: launch amcl node
  ```bash
  source devel/setup.bash 
  roslaunch my_robot amcl.launch
  ```

## Testing

### Method 1: 

Sending a `2D Nav Goal` from RViz. The `move_base` will try to navigate your robot based on the localization.
The robot will further perform the localization based on the new observation and the odometry. 

* Click on the `2D Nav Goal` Button in the toolbar, then click and drag on the map to send the goal to the robot. The robot should start moving and localize itself in the process.
* You could give the robot an initial position estimate on the map using a `2D pose estimate`

### Method 2: Use `teleop` Node

* Open another terminal and launch the `teleop` script: 

``` bash
rosrun teleop_twist_keyboard tleop_twist_keyboard.py
```
* Control the robot using commands from keyboard
## Suggestions

* It is recommended to update and upgrade your environment before running the code

```bash 

sudo apt-get update && sudo apt-get upgrade -your

```

* Recheck mapping in the amcl.launch files incase of errors while launching it. 

``` bash
<remap to="scan" from ="my_robot/laser/scan"/>
```
