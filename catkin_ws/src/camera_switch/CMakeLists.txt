cmake_minimum_required(VERSION 2.8.3)
project(camera_switch)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  diagnostic_msgs
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(camera_switch src/camera_switch.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(camera_switch
  ${catkin_LIBRARIES}
)
