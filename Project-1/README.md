## Project 1: Build My World

### Project Overview

In this project a simulation world has been created which shall be used for all the upcoming projects.

![image](https://user-images.githubusercontent.com/52028092/205459721-bae1f529-7d76-45b1-bde6-3337165f83b7.png)


The project targets to achieve the following: 

1. Build a single floor wall structure using the Building Editor tool in Gazebo. Apply at least one feature, one color, and optionally one texture to your structure. Make sure there's enough space between the walls for a robot to navigate.
2. Model any object of your choice using the Model Editor tool in Gazebo. Your model links should be connected with joints.
3. Import your structure and two instances of your model inside an empty Gazebo World.
4. Import at least one model from the Gazebo online library and implement it in your existing Gazebo world.
5. Write a C++ World Plugin to interact with your world. Your code should display “Welcome to ’s World!” message as soon as you launch the Gazebo world file.

### Dependencies

Please make sure the following dependencies are met before running the project: 

1. Gazebo >= 7.0
2. ROS Kinetic
3. make >= 4.1(mac,linux) , 3.81 (Windows)
    - Linux: make is installed by default 
    - Mac: Install [Xcode](https://developer.apple.com/xcode/features/)
    - Windows: [Installation Instructions](https://gnuwin32.sourceforge.net/packages/make.htm)
    
4. gcc/gcc++ >= 5.4
    - Linux: gcc/gcc++ is installed by default
    - Mac: Install [X Code](https://developer.apple.com/xcode/features/)
    - Windows: Install [MinGW](https://sourceforge.net/projects/mingw/)
    
### Setup

Step 1: Make sure the dependencies are met
Step 2: Open bash/terminal and clone the project repository
Step 3: Execute the following line in the command line to update info and upgrade the outdated packages and dependencies on your system.
```bash
sudo apt-get update && sudo apt-get upgrade -y
```

Step 4: Build and run the project

### Directory Structure

``` bash
.Project 1
├── CMakeLists.txt
├── model
│   ├── Building
│   │   ├── model.config
│   │   └── model.sdf
│   └── myrobot
│       ├── model.config
│       └── model.sdf
├── script
│   └── hello.cpp
└── world
    └── myworld
```

Folder and File Descriptions: 

1. **[model](model)** : Contains all the model files
    - **[Building](model/Building)**: Building structure built using the building editor of Gazebo
    - **[myrobot](model/myrobot)**: Robot model built using model editor of Gazebo
3. **[script](script)**: Contains a Gazebo world plugin C++ script
4. **[world](world)**: Contains the Gazebo World file written in the SDF Format  
5. **[CMakeLists.txt](CMakeLists.txt)**: File to link the C++ code to the libraries

### Running the Project

Step 1: Clone the repository
```bash
git clone https://github.com/Ayushman-Choudhuri/Udacity-Robotics-Software-Engineer-Nanodegree
```
Step 2: At the top level of the project repository, create a build directory
```bash
mkdir build && cd build
```
Step 3: In the build directory, compile your code with 
```bash
cmake .. && make
```
Step 4: Export your plugin folder in the terminal so your world file can find it 
```bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/workspace/Project\ 1/build
```

Step 5: launch the world file in Gazebo to  load both the world and plugin file
```bash
cd /home/workspace/Project\ 1/world/

gazebo myworld
```
### Suggestions

It is recommended to update and upgrade your working environment before running the code:   

```bash
sudo apt-get update && sudo apt-get upgrade -y
```

