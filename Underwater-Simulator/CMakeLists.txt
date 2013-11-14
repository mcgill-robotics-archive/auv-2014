cmake_minimum_required(VERSION 2.8.3)
project(Underwater-Simulator)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(move_model src/move_model.cc)
target_link_libraries(move_model ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(launch_torpedo src/launch_torpedo.cc)
target_link_libraries(launch_torpedo ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)
