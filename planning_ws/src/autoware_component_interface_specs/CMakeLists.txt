cmake_minimum_required(VERSION 3.10)
project(autoware_component_interface_specs)

find_package(catkin REQUIRED COMPONENTS
  autoware_adapi_v1_msgs autoware_control_msgs autoware_localization_msgs
  autoware_map_msgs autoware_perception_msgs autoware_planning_msgs
  autoware_system_msgs autoware_vehicle_msgs geometry_msgs nav_msgs roscpp sensor_msgs)

add_library(${PROJECT_NAME} src/names.cpp)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS
  autoware_adapi_v1_msgs autoware_control_msgs autoware_localization_msgs
  autoware_map_msgs autoware_perception_msgs autoware_planning_msgs
  autoware_system_msgs autoware_vehicle_msgs geometry_msgs nav_msgs roscpp sensor_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_qos_contract test/qos_latch_runtime.test
    test/gtest_main.cpp
    test/qos_contract_test.cpp
    test/qos_latch_runtime_test.cpp)
  if(TARGET test_qos_contract)
    add_dependencies(test_qos_contract
      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test_qos_contract ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# Public headers use <autoware/...>; export them from the global include root so
# the installed include directory advertised by catkin resolves that spelling.
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
