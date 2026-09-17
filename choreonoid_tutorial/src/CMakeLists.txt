choreonoid_add_simple_controller(StandController StandController.cpp)

choreonoid_add_simple_controller(ROSChoreonoidBridgeController ROSChoreonoidBridgeController.cpp)
target_link_libraries(ROSChoreonoidBridgeController ${catkin_LIBRARIES})
add_dependencies(ROSChoreonoidBridgeController
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# inference
# Pass -DTorch_DIR=... or add LibTorch to CMAKE_PREFIX_PATH.
find_package(Torch REQUIRED)
separate_arguments(TORCH_COMPILE_OPTIONS NATIVE_COMMAND "${TORCH_CXX_FLAGS}")
choreonoid_add_simple_controller(InferenceController InferenceController.cpp)
target_link_libraries(InferenceController ${CHOREONOID_BASE_LIBRARIES} "${TORCH_LIBRARIES}")
set_property(TARGET InferenceController PROPERTY CXX_STANDARD 17)
choreonoid_add_simple_controller(InferenceControllerSB InferenceControllerSB.cpp)
target_link_libraries(InferenceControllerSB ${CHOREONOID_BASE_LIBRARIES} "${TORCH_LIBRARIES}")
set_property(TARGET InferenceControllerSB PROPERTY CXX_STANDARD 17)

foreach(controller InferenceController InferenceControllerSB)
  target_compile_options(${controller} PRIVATE ${TORCH_COMPILE_OPTIONS})
  set_target_properties(${controller} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
endforeach()
