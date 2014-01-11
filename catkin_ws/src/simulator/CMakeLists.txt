cmake_minimum_required(VERSION 2.8.3)
project(simulator)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  rospy
  std_msgs
  message_generation
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS} ${roscpp_LIBRARY_DIRS})
include_directories(${roscpp_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
include(cmake/rospack.cmake)


add_message_files(
  FILES
  ThrusterForces.msg
)

generate_messages()

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    message_runtime
)

add_library(move_model src/move_model.cc)
target_link_libraries(move_model ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(launch_torpedo src/launch_torpedo.cc)
target_link_libraries(launch_torpedo ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


add_library(create_torpedo src/create_torpedo.cc)
target_link_libraries(create_torpedo ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#add_library(robot src/robot.cc)
#target_link_libraries(robot ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

set(CMAKE_INSTALL_RPATH "${roscpp_LIBRARY_DIRS}")
add_library(robot SHARED src/robot.cc)
set_target_properties(robot PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
set_target_properties(robot PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")

target_link_libraries(robot ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
