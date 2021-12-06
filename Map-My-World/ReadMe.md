In this project, I create a 2D occupancy grid and 3D octomap from the world which I built from previous project. 
The map is created through interacting with rtabmap_ros package, which is a ROS wrapper for interacting with RTAB-MAP.
<br>
<br>
To view the map, we'll need to use rtabmap-databaseViewer tool as the map data is contained within the rtabmap.db file (which is also attached in the project repo).
<br>
<br>
In this project, the following edits were made to make sure previous projects can align with this project's requirements:
* Modify my_robot.xacro to update the sensors
* Modify my_robot.gazebo file to use new shared object file
* Create mapping.launch under launch folder to remap sensors data to correct topics
* Finally, clone the ros teleop package so that I can drive the bots around the house I created, while the rtab-map package is mapping the world in the backend.
