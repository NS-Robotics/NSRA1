#! /bin/bash

git pull origin master
cd ~/NS-RobotArm/robot_ws/
catkin build
source install/setup.bash
