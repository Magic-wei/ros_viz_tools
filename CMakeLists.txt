cmake_minimum_required(VERSION 2.8.3)
project(ros_viz_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
set(catkin_deps
  geometry_msgs
  roscpp
  std_msgs
  visualization_msgs
  tf2
)

find_package(catkin REQUIRED COMPONENTS
  ${catkin_deps}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${catkin_deps}
)

# include
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# library
add_library(${PROJECT_NAME}
  src/ros_viz_tools.cpp
  src/color.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# demo node
add_executable(demo_node
  src/demo_node.cpp
)

target_link_libraries(demo_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
