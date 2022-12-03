## Brief Description of the files and packages


1. view_navigation.launch: Launch rviz with specify rviz configuration file

2. add_markers.cpp: C++ script, communicate with pick_objects node and control the marker appearance to simulate object pick up and drop off

3. pick_objects.cpp: C++ script, communicate with add_markers node and command the robot to pick up the object

4. home_service_rvizConfig.rviz: rvizConfig file for home service robot demo which contained markers option

5. add_marker.sh: Shell script file to deploy a turtlebot inside your environment, model a virtual object with markers in rviz.

6. home_service.sh: Shell script file to deploy a turtlebot inside your environment, simulate a full home service robot capable of navigating to pick up and deliver virtual objects.

7. pick_objects.sh: Shell script file to deploy a turtlebot inside your environment, communicate with the ROS navigation stack and autonomously send successive goals for your robot to reach.

8. test_navigation.sh: Shell script file to deploy a turtlebot inside your environment, pick two different goals and test your robot's ability to reach them and orient itself with respect to them.

9. test_slam.sh: Shell script file to deploy a turtlebot inside your environment, control it with keyboard commands, interface it with a SLAM package, and visualize the map in rviz

10. CMakeLists.txt: File to link the C++ code to libraries.
