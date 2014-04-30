cmake_minimum_required(VERSION 2.8.3)
project(front_end)

find_package(catkin REQUIRED COMPONENTS
    )

catkin_package(
    CATKIN_DEPENDS 
    std_msgs 
    sensor_msgs 
    computer_vision 
    controls 
    geometry_msgs
    status 
    blinky 
    rospy
    )

