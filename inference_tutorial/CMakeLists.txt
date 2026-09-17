cmake_minimum_required(VERSION 3.10)
project(inference_tutorial)

find_package(catkin REQUIRED)
catkin_package()

# Pass -DTorch_DIR=... or add LibTorch to CMAKE_PREFIX_PATH.
find_package(Torch REQUIRED)
separate_arguments(TORCH_COMPILE_OPTIONS NATIVE_COMMAND "${TORCH_CXX_FLAGS}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(inference inference.cpp)
target_link_libraries(inference "${TORCH_LIBRARIES}")
target_compile_options(inference PRIVATE ${TORCH_COMPILE_OPTIONS})
# Keep the external LibTorch library directory after installation.
set_target_properties(inference PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS inference RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS
  scripts/dump_cfgs_to_yaml.py
  scripts/dump_training_data.py
  scripts/sb_dump_training_data.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
