![Robot Starting Pose](https://github.com/huytrinhx/Robotics-ND/blob/main/Home-Service-Robot/screenshots/RobotStartingPose.JPG "Robot Starting Pose")

# Overview

This is the final robotics project in Udacity Robotics Software Engineer Nanodegree. In this, I put everything together I've learned so far about robotics: Gazebo, ROS, localization, mapping and path planning. The task in this project is to simulate a full home service robot capable of navigating to pick up and deliver a virtual objects in the Gazebo world.

In the world that I designed ( mimic the current house I live in :) ), the starting position of the robot has *[-4,0]* coordinates. The pickup zone is in my bedroom that has *[10,1]* coordinates. Initially, a green marker was displayed in this location.

Then the robot has to plan a path to this location.

![Navigating](https://github.com/huytrinhx/Robotics-ND/blob/main/Home-Service-Robot/screenshots/NavigatingToPickup.JPG "Robot is planning a path to pickup position")

Once arrived, the robot would sleep for a 5 seconds while the green marker disappeared to simulate picking up tasks.

![AtPickup](https://github.com/huytrinhx/Robotics-ND/blob/main/Home-Service-Robot/screenshots/AtPickup.JPG "Robot at pickup position")

After that, the next green marker appeared at the drop-off zone, which has *[-11,-4]* coordinates. 
Once reaching the drop-off, the green marker appeared back in the pickup zone.

![AtDropoff](https://github.com/huytrinhx/Robotics-ND/blob/main/Home-Service-Robot/screenshots/AtDropoff.JPG "Robot at dropoff position")

Overall where the robot started, picked up and dropped off were in 3 different room, which were connected with each other through a narrow common hallway. This is to demonstrate and test the system's ability to localize itself through a self-constructed map and constructing a path to the destination.

![MyWorldMap](https://github.com/huytrinhx/Robotics-ND/blob/main/Home-Service-Robot/screenshots/MySLAMMap.JPG "The map of my world generated through SLAM")

# Unit Testing

To test each function of the robot, we would need to launch several related nodes at the same time. To automate this mundane tasks, I've grouped the command in separate shell scripts (locate in /src/scripts folder).

- To test slam function and create an original map, run these commands:

`cd /home/workspace/catkin_ws`
`./src/scripts/test_slam.sh`

- To test navigation and test the bot's ability to reach a goal (given through RViz) in the map constructed through slam_gmapping package, run:

`./src/scripts/test_navigation.sh`

- To test the bot's ability to reach the pick up and drop off zone, run:

`./src/scripts/pick_objects.sh`

- To test whether the markers displayed correctly, enable this line in the main method of add_markers.cpp

![TestLine](https://github.com/huytrinhx/Robotics-ND/blob/main/Home-Service-Robot/screenshots/DisableThis.JPG "Disable/Enable this testing line")

then run:

`./src/scripts/add_markers.sh`

- Finally, to simulate the whole home service task with communication between pick_objects and add_markers nodes, disable this line

![TestLine](https://github.com/huytrinhx/Robotics-ND/blob/main/Home-Service-Robot/screenshots/DisableThis.JPG "Disable/Enable this testing line")

then run:

`./src/scripts/home_service.sh`

# Challenges overcome

Keep in mind there are more than one way to enable the communication between **pick_objects** and **add_markers**. In this project, I chose to implement a service request that the client in pick_objects to make a call to add_markers to signal the robot state (such as on the way to pickup zone, arrive pickup zone, heading to drop-off and done). But alternatively, we can have add_markers node subscribe to the robot's odometry to keep track of robot pose. 

In any case, there will be some discrepancies between the pose read by MoveBaseClient and the robot's own odometry. Therefore, I use tolerance number and set it to be 0.8 to account for the long distance robot had to travel. 

Also, by default, the slam_gmapping parameters are not great for the environment I designed, so I cloned two following repo to this project and edit these params from kinect_gmapping.launch.xml in the following directory

`\src\turtlebot_apps\turtlebot_navigation\launch\includes\gmapping`

# Dependencies:
- [gmapping](http://wiki.ros.org/gmapping)
- [turtlebot](http://wiki.ros.org/turtlebot_teleop)
- [turtlebot_simulator](http://wiki.ros.org/turtlebot_rviz_launchers)
- [turtlebot_interactions](http://wiki.ros.org/turtlebot_gazebo)
- [turtlebot_apps](https://github.com/turtlebot/turtlebot_apps)
- [turtlebot_msgs](https://github.com/turtlebot/turtlebot_msgs)
