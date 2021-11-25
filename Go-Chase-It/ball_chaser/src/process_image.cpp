#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv)) {
        ROS_ERROR("Failed to call service drive bot");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    bool image_found = false;
    int region =  0; 
    // We will divide the image into 5 portions
    // With the center regions occupy the the center 1/5th
    // Left: left 2/5th and Right: right 2/5th
    int grid_count = 5;
    int left_boundary = int(img.step / grid_count) * 2;
    int right_boundary = img.step - left_boundary;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    for (int i = 0; i < img.height * img.step; i++) {
        if (img.data[i] == white_pixel) {
            image_found = true;
            int white_ball_col = i % img.step;
            if (white_ball_col > left_boundary && white_ball_col < right_boundary) {
                region = 2; // map 2 to middle
            } else if (white_ball_col <= left_boundary) {
                region = 1; // map 1 to left
            } else {
                region = 3; // map 3 to right
            }
            ROS_INFO("White ball found in the region : %i -- 1:left,2:middle,3:right",region);
            break;
        }
    }
    // Request a stop when there's no white ball seen by the camera
    if (!image_found) {
       ROS_INFO("White ball not found in the image - Set the velocity to 0");
       drive_robot(0.0,0.0);
    }
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    switch (region) {
        case 2:
            // Move forward with vel 0.4
            drive_robot(0.4,0.0);
			break;
        case 1:
            // Turn left , very small velocity
            drive_robot(0.05, 0.1);
			break;
        case 3:
            // Turn right, very small velocity
            drive_robot(0.05, -0.1);
			break;
    }

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
