![MyWorld](https://github.com/huytrinhx/Robotics-ND/blob/main/Build-My-World/screenshots/MyWorld%26MyRobot.JPG "My world and the 4-wheeled robot")

## Overview

In this project, I learned to work with Model Editor and Building Editor in Gazebo. In particular, I learned how to embed a mobile robot I designed into a world structure.

## Project Structure

```
    .Build-My-World                        # Build My World Project 
    ├── model                          # Model files 
    │   ├── my_world
    │   │   ├── model.config
    │   │   ├── model.sdf
    │   ├── my_home_robot
    │   │   ├── model.config
    │   │   ├── model.sdf
    ├── script                         # Gazebo World plugin C++ script      
    │   ├── welcome_message.cpp
    ├── world                          # Gazebo main World containing models 
    │   ├── myworld
    ├── CMakeLists.txt                 # Link libraries 
    └──                              
```

## Quickstart

- To run open the world in gazebo, cd to the world folder and run:

`gazebo myworld`

- I'm using Udacity-provided Ubuntu VM. However, please look at instruction to install the environment on your local VMWare [here](https://classroom.udacity.com/nanodegrees/nd209/parts/37901367-6241-4258-8bb9-68838da05163/modules/d619b498-7960-49be-9e66-53ea20c39a8f/lessons/a8c605d3-977c-4b01-a667-31b4b2640be4/concepts/4a8a1de6-1721-492d-9533-fb84175944e7)
