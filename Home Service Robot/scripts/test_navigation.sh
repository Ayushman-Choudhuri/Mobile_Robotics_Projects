#!/bin/sh

#Define workspace variable
path_catkin_ws="/home/workspace/catkin_ws"

#Open the workspace , source and launch turtlebot_world.launch
xterm -e "cd ${path_catkin_ws} && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch" &

sleep 5

# Open the workspace, source and launch amcl_demo.launch 
xterm -e "cd ${path_catkin_ws} && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch" &

sleep 5

# Open the workspace, source and launch view_navigation.launch 
xterm -e "cd ${path_catkin_ws} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch"
