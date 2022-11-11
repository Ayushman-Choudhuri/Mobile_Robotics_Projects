#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <array>
#include <algorithm>


// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float l_x, float a_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Setting Bot Velocity");
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = l_x;
    srv.request.angular_z = a_z;

    // Call the drive_bot service and pass the requested velocity values
    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive_bot");
}


// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    /*
        After logging the rostopic to .txt file the following info was obtained
        img.height = 800
        img.width = 800
        img.step = 2400 (img.height * img.width * 3 (RGB channel))
        img.data = array with 0-255 value to represent pixel value and its size is height*step 1,920,000
                   [r1,g1,b1,r2,g2,b2...,rn,gn,gn] where n is the height*step and the values represent RGB (3 channels)
    */


    int white_pixel_val = 255; 

    int left_white_pixel_count = 0;
    int center_white_pixel_count = 0;
    int right_white_pixel_count = 0;

    
    int left_boundary = (int)(0.30*img.width); // 0 to 30% of image width
    int right_boundary = (int)(0.70*img.width); // > 70% of image width


    for (size_t i = 0; i < (img.height * img.step); i+=3) // 3 to loop over each pixel as RGB present
    {
        // check R G B channel for white pixel
        if ((img.data[i] == white_pixel_val) && (img.data[i+1] == white_pixel_val) && (img.data[i+2] == white_pixel_val))
        {
            int pixel_id = (i/3) % img.width; // 0 to 800
            if (pixel_id < left_boundary) // pixel belongs to left region
            {
                left_white_pixel_count++; // left counter
            }
            else if (pixel_id > right_boundary) // pixel belongs to right region
            {
                right_white_pixel_count++; // right counter
            }
            else // pixel belongs to center region
            {
                center_white_pixel_count++; // center counter
            }
        }
    }

    // Move Left
    if ((left_white_pixel_count> right_white_pixel_count) && (left_white_pixel_count> center_white_pixel_count))
    {
        drive_robot(0.5, 0.5);
    }
    // Move Right
    else if ((right_white_pixel_count > left_white_pixel_count) && (right_white_pixel_count > center_white_pixel_count))
    {
        drive_robot(0.5, -0.5);
    }
    // Move Forward
    else if ((center_white_pixel_count > left_white_pixel_count) && (center_white_pixel_count > right_white_pixel_count))
    {
        drive_robot(1.0, 0.0);
    }
    else // stop
    {
        drive_robot(0.0, 0.0);
    }
    return;
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}