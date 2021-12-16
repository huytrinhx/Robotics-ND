# Overview

In this project, I get deeper into modifying the appearance of my_robot by editing the robot URDF file directly. 
Editting the color and make the wheel aligned with the chassis (after changing its dimensions and radius) will create some difference in how the fast/slow the robot drive/turn.

![InitialPose](https://github.com/huytrinhx/Robotics-ND/blob/main/Go-Chase-It/screenshots/InitialPose.JPG "Robot with bigger wheel, wood chassis and the white ball")

Then, I learn to create a ball_chaser package to hold the C++ nodes. In particular:

- The drive_bot will provide a ball_chaser/command_robot service to drive the robot by controlling its linear x and angular z velocities. The service should publish to the wheel joints and return back the requested velocies
- The process_image will read the robot's camera image, analyze it to determine the presence and position of a white ball. If a white ball exists in the image, the node request a service via a client to drive the robot towards it

# Project Structure

   .Go-Chase-It                          # Go Chase It Project
    ├── my_robot                       # my_robot package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── robot_description.launch
    │   │   ├── world.launch
    │   ├── meshes                     # meshes folder for sensors
    │   │   ├── hokuyo.dae
    │   ├── urdf                       # urdf folder for xarco files
    │   │   ├── my_robot.gazebo
    │   │   ├── my_robot.xacro
    │   ├── world                      # world folder for world files
    │   │   ├── <yourworld>.world
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
    ├── ball_chaser                    # ball_chaser package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── ball_chaser.launch
    │   ├── src                        # source folder for C++ scripts
    │   │   ├── drive_bot.cpp
    │   │   ├── process_images.cpp
    │   ├── srv                        # service folder for ROS services
    │   │   ├── DriveToTarget.srv
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info                  
    └──                              

# Quickstart

- Copy the src folder into your catkin workspace and then run:

`catkin_make`

- Launch the world and Rviz by this command in the current terminal:

`roslaunch my_robot world.launch`

You should see the robot, the world, the white ball showing up

In Rviz, you should see the robot, the camera image and the lidar scan of the ball and the world in front.

![Rviz](https://github.com/huytrinhx/Robotics-ND/blob/main/Go-Chase-It/screenshots/RVIZ.JPG "Make sure you can see the white ball in the camera image and point cloud in lidar scan")


- Launch the both drive_bot node and process_image node by this command in a new terminal:

`roslaunch ball_chaser ball_chaser.launch`

Now, you will see the robot drive toward the ball and keep kicking the ball until the ball is out of the camera range.

![Hitting the ball](https://github.com/huytrinhx/Robotics-ND/blob/main/Go-Chase-It/screenshots/ChasingWhiteBall.JPG "Robot hits the ball")
