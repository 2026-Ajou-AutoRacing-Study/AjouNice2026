cmake_minimum_required(VERSION 3.13)
project(autoware_behavior_path_start_planner_module)
find_package(catkin REQUIRED COMPONENTS roscpp pluginlib visualization_msgs
  autoware_behavior_path_planner_common autoware_freespace_planning_algorithms
  autoware_motion_utils autoware_rtc_interface autoware_utils autoware_lanelet2_utils
  autoware_vehicle_info_utils autoware_deprecated_boundary_departure_checker)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(autoware_behavior_path_lane_change_module REQUIRED CONFIG)
set(CMAKE_CXX_STANDARD 17)
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pluginlib visualization_msgs autoware_behavior_path_planner_common
  autoware_freespace_planning_algorithms autoware_motion_utils autoware_rtc_interface
  autoware_utils autoware_lanelet2_utils autoware_vehicle_info_utils
  autoware_deprecated_boundary_departure_checker)
add_library(${PROJECT_NAME} SHARED
  src/start_planner_module.cpp src/manager.cpp src/pull_out_planner_base.cpp
  src/freespace_pull_out.cpp src/clothoid_pull_out.cpp src/geometric_pull_out.cpp
  src/shift_pull_out.cpp src/data_structs.cpp src/util.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PRIVATE -include ${CMAKE_CURRENT_SOURCE_DIR}/include/autoware/behavior_path_start_planner_module/ros1_logging.hpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} fmt::fmt)
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
if(CATKIN_ENABLE_TESTING)
  find_package(autoware_test_utils REQUIRED)
  find_package(yaml-cpp REQUIRED)
  option(EXPORT_TEST_PLOT_FIGURE "Enable original optional test plots" OFF)
  file(GLOB TEST_SOURCES test/test_*.cpp)
  catkin_add_gtest(test_${PROJECT_NAME} ${TEST_SOURCES} test/src/start_planner_test_helper.cpp test/src/ros1_main.cpp)
  # Focused diagnostic target; the full original test target below remains
  # mandatory in ALL, including the not-yet-ported Planning Factor host test.
  catkin_add_gtest(test_original_pull_out_algorithms
    test/test_shift_pull_out.cpp test/test_geometric_pull_out.cpp
    test/test_clothoid_pull_out.cpp test/test_freespace_pull_out.cpp
    test/src/start_planner_test_helper.cpp test/src/ros1_main.cpp)
  foreach(test_target test_${PROJECT_NAME} test_original_pull_out_algorithms)
    if(TARGET ${test_target})
      target_include_directories(${test_target} PRIVATE test/include ${autoware_test_utils_INCLUDE_DIRS})
      target_compile_options(${test_target} PRIVATE -include ${CMAKE_CURRENT_SOURCE_DIR}/include/autoware/behavior_path_start_planner_module/ros1_logging.hpp)
      target_compile_definitions(${test_target} PRIVATE
        AW149_START_TEST_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}"
        AW149_BPP_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../autoware_behavior_path_planner/config")
      target_link_libraries(${test_target} ${PROJECT_NAME} ${autoware_test_utils_LIBRARIES} yaml-cpp)
      if(EXPORT_TEST_PLOT_FIGURE)
        find_package(autoware_pyplot REQUIRED)
        find_package(pybind11 REQUIRED)
        target_compile_definitions(${test_target} PRIVATE EXPORT_TEST_PLOT_FIGURE)
        target_include_directories(${test_target} PRIVATE ${autoware_pyplot_INCLUDE_DIRS})
        target_link_libraries(${test_target} ${autoware_pyplot_LIBRARIES} pybind11::embed)
      endif()
    endif()
  endforeach()
  if(TARGET test_${PROJECT_NAME})
    target_include_directories(test_${PROJECT_NAME} PRIVATE test/include ${autoware_test_utils_INCLUDE_DIRS})
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${autoware_test_utils_LIBRARIES})
    add_custom_target(build_original_start_tests ALL DEPENDS test_${PROJECT_NAME})
  endif()
endif()
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config test_data DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

find_package(autoware_utils_rclcpp REQUIRED)
include_directories(BEFORE ${autoware_utils_rclcpp_INCLUDE_DIRS})
