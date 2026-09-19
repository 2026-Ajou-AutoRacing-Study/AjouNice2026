cmake_minimum_required(VERSION 3.10)
project(autoware_component_interface_utils)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  autoware_adapi_v1_msgs
  autoware_component_interface_specs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs std_srvs autoware_adapi_v1_msgs autoware_component_interface_specs
  CFG_EXTRAS autoware_component_interface_utils-extras.cmake
)

include_directories(include ${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_component_interface_utils test/main.cpp test/test_component_interface_utils.cpp test/ros1_wrapper_test.cpp)
  if(TARGET test_component_interface_utils)
    target_link_libraries(test_component_interface_utils ${catkin_LIBRARIES})
  endif()
endif()

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(FILES cmake/autoware_component_interface_utils-extras.cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
