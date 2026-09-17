cmake_minimum_required(VERSION 3.10)
project(choreonoid_tutorial)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  sensor_msgs
  trajectory_msgs
)
# Choreonoid is a plain CMake package, not a catkin component.
find_package(Choreonoid REQUIRED)

add_message_files(FILES RobotObservation.msg)
generate_messages(DEPENDENCIES std_msgs sensor_msgs trajectory_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs trajectory_msgs
)

set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
include_directories(${catkin_INCLUDE_DIRS})
add_subdirectory(src)
