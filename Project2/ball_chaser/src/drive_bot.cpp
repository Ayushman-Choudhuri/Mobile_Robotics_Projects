#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// A handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities


bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
  ROS_INFO("DriveToTargetRequest received - linear_x: %1.2f, angular_z: %1.2f", (float)req.linear_x, (float)req.angular_z);

  geometry_msgs::Twist cmd_vel_;

  cmd_vel_.linear.x = req.linear_x; // both of them are float64
  cmd_vel_.angular.z = req.angular_z; // both of them are float64
  
  motor_command_publisher.publish(cmd_vel_);

  res.msg_feedback = "Velocity cmd sent: lin_x = " + std::to_string(cmd_vel_.linear.x) + ", ang_z = " + std::to_string(cmd_vel_.angular.z);

  ROS_INFO_STREAM(res.msg_feedback);
  

  return true;

}


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function

    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    
    ROS_INFO("Ready to send command velocities");

   
    // Handle ROS communication events
    ros::spin();

    return 0;
}