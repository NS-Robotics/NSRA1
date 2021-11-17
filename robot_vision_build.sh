#! /bin/bash

git pull origin master
cd ~/NS-RobotArm/robot_ws/
catkin build nsra_robot_vision
source install/setup.bash
