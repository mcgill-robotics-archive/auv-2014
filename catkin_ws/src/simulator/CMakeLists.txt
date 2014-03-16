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

find_package(Boost REQUIRED COMPONENTS system)

link_directories(${GAZEBO_LIBRARY_DIRS} ${roscpp_LIBRARY_DIRS})

include_directories(${roscpp_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

include(cmake/rospack.cmake)


add_message_files(
  FILES
  ThrusterForces.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
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

set(CMAKE_INSTALL_RPATH "${roscpp_LIBRARY_DIRS}")
add_library(robot SHARED src/robot.cc)
add_dependencies(robot ${PROJECT_NAME}_generate_messages_cpp)
set_target_properties(robot PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
set_target_properties(robot PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")

target_link_libraries(robot ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(tf_broadcaster_simulator src/tf_broadcaster_simulator.cpp)
target_link_libraries(tf_broadcaster_simulator ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(tf_broadcaster_simulator ${catkin_EXPORTED_TARGETS})

# gtest
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
catkin_add_gtest(test_robot test/test_robot.cc)
target_link_libraries(test_robot robot ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(gazeboDepthEstimator src/gazeboDepthEstimator.cpp)
target_link_libraries(gazeboDepthEstimator ${catkin_LIBRARIES})
add_dependencies(gazeboDepthEstimator ${catkin_EXPORTED_TARGETS})
