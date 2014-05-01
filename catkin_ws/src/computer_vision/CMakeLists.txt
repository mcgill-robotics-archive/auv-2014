cmake_minimum_required(VERSION 2.8.3)
project(computer_vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  message_generation
  tf
  planner
)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  VisibleObjectData.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES computer_vision
  CATKIN_DEPENDS roscpp std_msgs message_runtime planner
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(VisibleObject src/VisibleObject.cpp)
target_link_libraries(VisibleObject ${catkin_LIBRARIES})
add_dependencies(VisibleObject ${catkin_EXPORTED_TARGETS})

add_library(CVNode src/CVNode.cpp)
target_link_libraries(CVNode ${catkin_LIBRARIES})

add_library(Gate src/Gate.cpp)
target_link_libraries(Gate VisibleObject)
target_link_libraries(Gate ${catkin_LIBRARIES})

add_library(Buoy src/Buoy.cpp)
target_link_libraries(Buoy VisibleObject)
target_link_libraries(Buoy ${catkin_LIBRARIES})
rosbuild_add_compile_flags(Buoy -std=c++0x)

add_library(MarkerTarget src/MarkerTarget.cpp)
target_link_libraries(MarkerTarget VisibleObject)
target_link_libraries(MarkerTarget ${catkin_LIBRARIES})

catkin_add_gtest(marker_target_test tests/marker_target_test.cpp)
if(TARGET marker_target_test)
   target_link_libraries(marker_target_test MarkerTarget)
endif()

add_library(LineTarget src/LineTarget.cpp)
target_link_libraries(LineTarget VisibleObject)
target_link_libraries(LineTarget ${catkin_LIBRARIES})

catkin_add_gtest(line_target_test tests/line_target_test.cpp)
if(TARGET line_target_test)
   target_link_libraries(line_target_test LineTarget)
endif()

add_executable(PlannerSimulator src/PlannerSimulator.cpp)
target_link_libraries(PlannerSimulator ${catkin_LIBRARIES})

add_executable(cv_down_node src/DownCVNode.cpp)
target_link_libraries(cv_down_node CVNode)
target_link_libraries(cv_down_node VisibleObject)
target_link_libraries(cv_down_node MarkerTarget)
target_link_libraries(cv_down_node LineTarget)
target_link_libraries(cv_down_node ${catkin_LIBRARIES})

add_executable(cv_front_node src/FrontCVNode.cpp)
target_link_libraries(cv_front_node CVNode)
target_link_libraries(cv_front_node VisibleObject)
target_link_libraries(cv_front_node Gate)
target_link_libraries(cv_front_node Buoy)
target_link_libraries(cv_front_node ${catkin_LIBRARIES})
rosbuild_add_compile_flags(cv_front_node -std=c++0x)

add_executable(CVTestNode src/CVTestNode.cpp)
target_link_libraries(CVTestNode ${catkin_LIBRARIES})

add_executable (distanceCalculator src/distanceCalculator.cpp)
target_link_libraries(distanceCalculator ${catkin_LIBRARIES})

# This makes sure message headers are generated before being used
add_dependencies(CVNode computer_vision_gencpp)
add_dependencies(CVTestNode computer_vision_gencpp)
add_dependencies(distanceCalculator ${catkin_EXPORTED_TARGETS})
