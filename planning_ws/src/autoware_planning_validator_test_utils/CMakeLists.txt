cmake_minimum_required(VERSION 3.0.2)
project(autoware_planning_validator_test_utils)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs nav_msgs autoware_planning_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs autoware_planning_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/planning_validator_test_utils.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_planning_validator_test_utils test/test_planning_validator_test_utils.cpp)
  if(TARGET test_planning_validator_test_utils)
    target_link_libraries(test_planning_validator_test_utils ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
