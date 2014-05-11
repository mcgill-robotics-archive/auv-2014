#!/bin/bash

echo "Launching sensor nodes";

roslaunch ~/McGill_RoboSub_2014/catkin_ws/launch/arduino.launch;
roslaunch computer_vision camera_down.launch;
roslaunch computer_vision camera_front_left.launch;
roslaunch computer_vision camera_front_right.launch;
roslaunch viso2_ros demo.launch
