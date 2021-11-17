#!/bin/bash
cd robot_ws/
source install/setup.bash
rosservice call /axis_requested_state 1 3
sleep 0.1
rosservice call /axis_requested_state 2 3
sleep 0.1
rosservice call /axis_requested_state 3 3
echo "Axis 1, 2 and 3 callibration: Press any key to continue"
read -n 1
rosservice call /axis_requested_state 1 8
sleep 0.1
rosservice call /axis_requested_state 2 8
sleep 0.1
rosservice call /axis_requested_state 3 11
echo "Axis 3 homing: Press any key to continue"
read -n 1
rosservice call /axis_requested_state 3 8
sleep 0.1
rosservice call /axis_requested_position 3 0.0
sleep 3
rosservice call /axis_requested_state 4 3
sleep 0.1
rosservice call /axis_requested_state 5 3
sleep 0.1
rosservice call /axis_requested_state 6 3
echo "Axis 5 callibration: Press any key to continue"
read -n 1
rosservice call /axis_requested_state 5 11
echo "Axis 5 homing, axis 4 and 6 closed loop: Press any key to continue"
read -n 1
rosservice call /axis_requested_state 5 8
sleep 0.1
rosservice call /axis_requested_position 5 0.0
sleep 0.1
rosservice call /axis_requested_state 4 8
sleep 0.1
rosservice call /axis_requested_state 6 8
