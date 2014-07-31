cmake_minimum_required(VERSION 2.8.3)
project(planner)

 find_package (catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   geometry_msgs
   simulator
   message_generation
   blinky
   robosub_msg
   hydrophones
 )

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files (
  FILES
  setPoints.msg
  ValueControl.msg
  CurrentCVTask.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages (
  DEPENDENCIES
  std_msgs
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
   include/
   ${catkin_INCLUDE_DIRS}
 )

 add_executable(planner src/Planner.cpp
                        src/Task.cpp
                        src/StatusUpdater.cpp
                        src/Task_Gate.cpp
                        src/Lost_Gate.cpp
                        src/Task_Lane.cpp
                        src/Lost_Lane.cpp
                        src/Task_Kill.cpp)

 target_link_libraries(planner ${catkin_LIBRARIES})
 add_dependencies(planner ${catkin_EXPORTED_TARGETS})
 add_dependencies(planner planner_generate_messages_cpp)
 add_dependencies(planner state_estimation_generate_messages_cpp)
target_link_libraries(planner boost_thread)  

 add_executable (CurrentCVTaskPublisher src/CurrentCVTaskPublisher.cpp)
 target_link_libraries(CurrentCVTaskPublisher ${catkin_LIBRARIES})
 add_dependencies(CurrentCVTaskPublisher ${catkin_EXPORTED_TARGETS})
 add_dependencies(CurrentCVTaskPublisher planner_generate_messages_cpp)

