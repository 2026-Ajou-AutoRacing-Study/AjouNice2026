cmake_minimum_required(VERSION 3.14)
project(autoware_utils_debug)

find_package(catkin REQUIRED COMPONENTS
  autoware_internal_debug_msgs
  autoware_internal_msgs
  autoware_utils_system
  diagnostic_msgs
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS autoware_internal_debug_msgs autoware_internal_msgs autoware_utils_system diagnostic_msgs roscpp std_msgs
)

add_library(${PROJECT_NAME} SHARED src/time_keeper.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CATKIN_PACKAGE_INCLUDE_DESTINATION}>
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_executable(time_keeper_logger_probe test/time_keeper_logger_probe.cpp)
  target_link_libraries(time_keeper_logger_probe ${PROJECT_NAME} ${catkin_LIBRARIES})
  catkin_add_gtest(test_${PROJECT_NAME}
    test/main.cpp
    test/adapter_contract.cpp
    test/time_keeper.cpp
  )
  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
