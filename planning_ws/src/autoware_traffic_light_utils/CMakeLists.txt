cmake_minimum_required(VERSION 3.0.2)
project(autoware_traffic_light_utils)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  lanelet2_core
  tf2
  tier4_perception_msgs
)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS autoware_perception_msgs lanelet2_core tf2 tier4_perception_msgs
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} SHARED src/traffic_light_utils.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
  catkin_add_gtest(test_traffic_light_utils ${TEST_SOURCES})
  if(TARGET test_traffic_light_utils)
    target_include_directories(test_traffic_light_utils PRIVATE src/include)
    # The upstream test sources declare TEST cases but no user main().  Catkin's
    # test target therefore needs the framework main supplied explicitly.
    target_link_libraries(test_traffic_light_utils ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)
  endif()
endif()
