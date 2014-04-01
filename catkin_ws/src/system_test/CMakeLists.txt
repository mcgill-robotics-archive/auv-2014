cmake_minimum_required(VERSION 2.8.3)
project(system_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  controls
)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES system_test
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(hardware_test src/hardware_test.cpp)
target_link_libraries(hardware_test ${catkin_LIBRARIES})

add_dependencies(hardware_test system_test_gencpp)
