# Robotics

 This repo comprises of projects completed while enrolling in Udacity's Robotics Software Engineer Nanodegree
 
## Build My World

- In this project, I learned to work with Model Editor and Building Editor in Gazebo. In particular, I learned how to embed a two-wheeled robot I designed into a world structure.

![MyWorld](https://github.com/huytrinhx/Robotics-ND/blob/main/Build-My-World/screenshots/MyWorld%26MyRobot.JPG "My world and the 4-wheeled robot")

## Go Chase It

- Based on the previous project, I now equiped the bot with a lidar and a camera. Then I created a package that will drive the robot toward a white ball in the world.

![Hitting the ball](https://udacity-reviews-uploads.s3.us-west-2.amazonaws.com/_attachments/345774/1637899565/Peek_2021-11-26_04-54.gif "Robot hits the ball")

## Where Am I

- In this project, I learned to utilize ROS AMCL (Adaptive Monte Carlo Localization) to accurately localize a mobile robot inside a map in the world. 

![Localize](https://github.com/huytrinhx/Robotics-ND/blob/main/Where-Am-I/screenshots/LocalizeItself.JPG)

## Map My World

- In real life, many environments do not have known maps. In this project, I learned to create a 2D occupancy grid and 3D octomap of the environment from sensor data using RTAB-Map package

![3d](https://github.com/huytrinhx/Robotics-ND/blob/main/Map-My-World/screenshots/3D-Map.JPG)

## Home Service Robot

- Finally, putting everything together, I simulated a full home service robot that finds a path to pickup zone and then travel to dropoff zone. 

![Navigating](https://github.com/huytrinhx/Robotics-ND/blob/main/Home-Service-Robot/screenshots/NavigatingToPickup.JPG "Robot is planning a path to pickup position")

# Extra Resources
### Gazebo
- [Solidworks Tutorial Video](https://www.youtube.com/watch?v=T7X_p_KMwus)
- [Solidworks Tutorial PDF](https://blogs.solidworks.com/teacher/wp-content/uploads/sites/3/WPI-Robotics-SolidWorks-to-Gazebo.pdf)
- [Create more floors to building](http://gazebosim.org/tutorials?cat=build_world&tut=building_editor#Addlevels)
- [Building Editor Best Practices](http://gazebosim.org/blog/buildingeditor)
- [Simple but detailed video on how to create a gazebo model using sdf specification](https://www.youtube.com/watch?v=3YhW04wIjEc&ab_channel=TheConstruct)
- [Detailed documentation of the SDF Format](http://sdformat.org/tutorials)
- [Adding colors to model](http://gazebosim.org/tutorials/?tut=ros_urdf#Materials:Usingpropercolorsandtextures)
- [Plugins gazebo supports](http://gazebosim.org/tutorials?tut=ros_gzplugins)
- [How to create sensor plugins](https://www.theconstructsim.com/create-a-ros-sensor-plugin-for-gazebo/)
- [Visual gazebo plugin](https://github.com/alexandrelheinen/vector-view)
### ROS
- [ROS Best Practices](https://github.com/leggedrobotics/ros_best_practices)
- [Top 5 Advanced Robotics Kit](https://linuxhint.com/top_5_advanced_robotics_kits/)
- [Robotic Maintenance and ROS](https://core.ac.uk/download/pdf/154676353.pdf)
- [ROS Tutorials](https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/)



