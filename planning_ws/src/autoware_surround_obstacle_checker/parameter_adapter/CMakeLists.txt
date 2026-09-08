cmake_minimum_required(VERSION 3.0.2)
project(aw149_surround_obstacle_checker_parameter_adapter)

find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package(INCLUDE_DIRS include)
include_directories(include ${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_surround_obstacle_checker_parameters test_surround_obstacle_checker_parameters.cpp)
  if(TARGET test_surround_obstacle_checker_parameters)
    target_link_libraries(test_surround_obstacle_checker_parameters ${catkin_LIBRARIES})
    target_compile_features(test_surround_obstacle_checker_parameters PUBLIC cxx_std_17)
  endif()
endif()
