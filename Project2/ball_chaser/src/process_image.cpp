#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <array>
#include <algorithm>


// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Setting Bot Velocity");
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

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
    int white_pixel = 255; // when R=255, G=255, B=255 implies white pixel

    int leftWhitePixelCounter = 0;
    int centerWhitePixelCounter = 0;
    int rightWhitePixelCounter = 0;

    // if image width is 800, then 1 - 239 is left, 240 - 560 is center, 561 - 800 is right
    int leftRegionBoundary = (int)(0.30*img.width); // 0 to 30% of image width
    int rightRegionBoundary = (int)(0.70*img.width); // > 70% of image width


    for (size_t i = 0; i < (img.height * img.step); i+=3) // 3 to loop over each pixel as RGB present
    {
        // check R G B channel for white pixel
        if ((img.data[i] == white_pixel) && (img.data[i+1] == white_pixel) && (img.data[i+2] == white_pixel))
        {
            int pixelId = (i/3) % img.width; // 0 to 800
            if (pixelId < leftRegionBoundary) // pixel belongs to left region
            {
                leftWhitePixelCounter++; // left counter
            }
            else if (pixelId > rightRegionBoundary) // pixel belongs to right region
            {
                rightWhitePixelCounter++; // right counter
            }
            else // pixel belongs to center region
            {
                centerWhitePixelCounter++; // center counter
            }
        }
    }

    // Move Left
    if ((leftWhitePixelCounter > rightWhitePixelCounter) && (leftWhitePixelCounter > centerWhitePixelCounter))
    {
        drive_robot(0.5, 0.5);
    }
    // Move Right
    else if ((rightWhitePixelCounter > leftWhitePixelCounter) && (rightWhitePixelCounter > centerWhitePixelCounter))
    {
        drive_robot(0.5, -0.5);
    }
    // Move Forward
    else if ((centerWhitePixelCounter > leftWhitePixelCounter) && (centerWhitePixelCounter > rightWhitePixelCounter))
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