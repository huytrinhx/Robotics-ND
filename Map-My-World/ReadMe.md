# Overview

In this project, I create a 2D occupancy grid and 3D octomap from the world which I built from previous project. 

The 2D grid map is on the left panel of the screenshot:
![2d](https://github.com/huytrinhx/Robotics-ND/blob/main/Map-My-World/screenshots/RTAB-View.JPG)

The 3D octomap after the mapping:
![3d](https://github.com/huytrinhx/Robotics-ND/blob/main/Map-My-World/screenshots/3D-Map.JPG)

The map is created through interacting with **rtabmap_ros package**, which is a ROS wrapper for interacting with RTAB-MAP.
<br>
<br>
To view the map, we'll need to use rtabmap-databaseViewer tool as the map data is contained within the rtabmap.db file (which is saved at `~/.ros/`).
<br>
<br>
In this project, the following edits were made to make sure previous projects can align with this project's requirements:

* Modify **my_robot.xacro** to update the sensors (so that rtabmap can interface with the robot's sensors)
* Modify **my_robot.gazebo** file to use new shared object file
* Create **mapping.launch** under launch folder to remap sensors data to correct topics
* Finally, clone the ros teleop package so that I can drive the bots around the house I created, while the rtab-map package is mapping the world in the backend.


# Quickstart

- Copy the src folder into your catkin workspace and then run:

`catkin_make`

- Launch the world and Rviz by this command in the current terminal:

`roslaunch my_robot world.launch`

- Launch rtabmap-ros package by this command in a new terminal:

`roslaunch my_robot mapping.launch`

- To drive the bot around and see mapping in action, run this command in a terminal:

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

Make sure to drive slowly and complete each room at least one circle so that loop closure can work.

Once done driving, we can open the rtabmap-view by launching this command:

`rtabmap-databaseViewer ~/.ros/rtabmap.db`





