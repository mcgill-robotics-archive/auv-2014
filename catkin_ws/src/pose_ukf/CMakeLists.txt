cmake_minimum_required(VERSION 2.8.3)
project(pose_ukf)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin 
  REQUIRED 
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)





catkin_package(
  CATKIN_DEPENDS  roscpp std_msgs geometry_msgs sensor_msgs
)



## Declare a cpp executable
add_executable(pose_ukf
  src/ros_pose.cpp 
  src/ukf.cpp
  src/matrix_utils.cpp
  src/rotation_vector_utils.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(pose_ukf ${catkin_LIBRARIES})




#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
include_directories(src)
catkin_add_gtest(${PROJECT_NAME}-test 
  test/test_ukf.cpp
  src/matrix_utils.cpp
  src/ukf.cpp
  src/rotation_vector_utils.cpp
)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
