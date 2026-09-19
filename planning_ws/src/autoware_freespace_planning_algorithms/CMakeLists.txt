cmake_minimum_required(VERSION 3.14)
project(autoware_freespace_planning_algorithms)

find_package(catkin REQUIRED COMPONENTS
  roscpp rosbag geometry_msgs nav_msgs std_msgs tf2 tf2_geometry_msgs
  autoware_utils_geometry autoware_utils_math autoware_vehicle_info_utils
  pybind11_catkin
)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(nlohmann_json REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES reeds_shepp rrtstar_core ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rosbag geometry_msgs nav_msgs std_msgs tf2 tf2_geometry_msgs
    autoware_utils_geometry autoware_utils_math autoware_vehicle_info_utils pybind11_catkin
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_compile_options(-std=c++17)
add_library(reeds_shepp src/reeds_shepp.cpp)
add_library(rrtstar_core src/rrtstar_core.cpp)
target_link_libraries(rrtstar_core reeds_shepp ${catkin_LIBRARIES} nlohmann_json::nlohmann_json)
add_library(${PROJECT_NAME} src/abstract_algorithm.cpp src/astar_search.cpp src/rrtstar.cpp)
target_link_libraries(${PROJECT_NAME} reeds_shepp rrtstar_core ${catkin_LIBRARIES})

pybind11_add_module(${PROJECT_NAME}_pybind SHARED scripts/bind/astar_search_pybind.cpp)
target_include_directories(${PROJECT_NAME}_pybind PRIVATE include)
target_link_libraries(${PROJECT_NAME}_pybind PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_pybind PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})

install(TARGETS reeds_shepp rrtstar_core ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ${PROJECT_NAME}_pybind
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
install(DIRECTORY include/autoware/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware)
catkin_python_setup()
install(PROGRAMS test/debug_plot.py scripts/example/example.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test test/src/test_freespace_planning_algorithms.cpp)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
  catkin_add_gtest(rrtstar_core_informed-test test/src/test_rrtstar_core_informed.cpp)
  target_link_libraries(rrtstar_core_informed-test rrtstar_core ${catkin_LIBRARIES} gtest_main)
endif()
