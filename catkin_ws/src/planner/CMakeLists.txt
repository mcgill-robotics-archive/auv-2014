cmake_minimum_required(VERSION 2.8.3)
project(planner)

 find_package (catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   geometry_msgs
   simulator
   message_generation
 )

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files (
   FILES
   MotorControl.msg
   ValueControl.msg
   setPoints.msg
 )

## Generate services in the 'srv' folder
# add_service_files (
#   FILES
# )

## Generate added messages and services with any dependencies listed here
 generate_messages (
   DEPENDENCIES
   std_msgs
   planner
   simulator
 )

###################################
## catkin specific configuration ##
###################################

 catkin_package (
   CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
 )

###########
## Build ##
###########

 include_directories (
   ${catkin_INCLUDE_DIRS}
 )
 add_executable(TestNodePleaseIgnore src/rosTest.cpp)
 target_link_libraries(TestNodePleaseIgnore ${catkin_LIBRARIES})

 add_executable(Planner src/Interface.cpp src/Loader.cpp src/Invoker.cpp src/TaskFactory.cpp src/Config.cpp include/pugixml/src/pugixml.cpp src/TaskConcrete.cpp src/Task.cpp src/Task_Gate.cpp )
 target_link_libraries(Planner ${catkin_LIBRARIES})

 include_directories(include ${catkin_INCLUDE_DIRS})


