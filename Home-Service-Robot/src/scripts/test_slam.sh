#!/bin/sh
export ROBOT_INITIAL_POSE="-x 4 -y 0.0 -z 0.0"
xterm -e " source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e " source devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm -e " source devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch" &
sleep 5
xterm -e " source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
