cmake_minimum_required(VERSION 3.0.2)
project(inference_tutorial)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES inference_tutorial
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

find_package(Torch REQUIRED)

# use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(inference inference.cpp)
target_link_libraries(inference "${TORCH_LIBRARIES}")

