cmake_minimum_required(VERSION 2.8.3)
project(computer_vision)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  VisibleObjectData.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES computer_vision
#  CATKIN_DEPENDS cpp ros rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS roscpp std_msgs message_runtime planner
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(computer_vision
#   src/${PROJECT_NAME}/computer_vision.cpp
# )

## Declare a cpp executable
# add_executable(computer_vision_node src/computer_vision_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(computer_vision_node computer_vision_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(computer_vision_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS computer_vision computer_vision_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_computer_vision.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(VisibleObject src/VisibleObject.cpp)
target_link_libraries(VisibleObject ${catkin_LIBRARIES})
add_dependencies(VisibleObject ${catkin_EXPORTED_TARGETS})

add_library(CVNode src/CVNode.cpp)
target_link_libraries(CVNode ${catkin_LIBRARIES})

add_library(CameraNode src/CameraNode.cpp)
target_link_libraries(CameraNode ${catkin_LIBRARIES})

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

add_library(Camera src/Camera.cpp)
target_link_libraries(Camera ${catkin_LIBRARIES})

add_executable(PlannerSimulator src/PlannerSimulator.cpp)
target_link_libraries(PlannerSimulator ${catkin_LIBRARIES})

add_executable(down_cv_node src/DownCVNode.cpp)
target_link_libraries(down_cv_node CVNode)
target_link_libraries(down_cv_node VisibleObject)
target_link_libraries(down_cv_node MarkerTarget)
target_link_libraries(down_cv_node LineTarget)
target_link_libraries(down_cv_node ${catkin_LIBRARIES})

add_executable(front_cv_node src/FrontCVNode.cpp)
target_link_libraries(front_cv_node CVNode)
target_link_libraries(front_cv_node VisibleObject)
target_link_libraries(front_cv_node Gate)
target_link_libraries(front_cv_node Buoy)
target_link_libraries(front_cv_node ${catkin_LIBRARIES})
rosbuild_add_compile_flags(front_cv_node -std=c++0x)

add_executable(CVTestNode src/CVTestNode.cpp)
target_link_libraries(CVTestNode ${catkin_LIBRARIES})

add_executable (distanceCalculator src/distanceCalculator.cpp)
target_link_libraries(distanceCalculator ${catkin_LIBRARIES})

# This makes sure message headers are generated before being used
add_dependencies(CameraNode computer_vision_gencpp)
add_dependencies(CVNode computer_vision_gencpp)
add_dependencies(CVTestNode computer_vision_gencpp)
add_dependencies(distanceCalculator ${catkin_EXPORTED_TARGETS})
