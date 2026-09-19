cmake_minimum_required(VERSION 3.0.2)
project(autoware_signal_processing)

add_compile_options(-std=c++17)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lowpass_filters
  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(lowpass_filters
  src/lowpass_filter_1d.cpp
  src/lowpass_filter.cpp
  src/butterworth.cpp)
target_link_libraries(lowpass_filters ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_signal_processing
    test/src/lowpass_filter_1d_test.cpp
    test/src/lowpass_filter_test.cpp
    test/src/butterworth_filter_test.cpp)
  if(TARGET test_signal_processing)
    target_include_directories(test_signal_processing PRIVATE test/include)
    target_link_libraries(test_signal_processing lowpass_filters gtest_main ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS lowpass_filters
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/autoware/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware)
install(DIRECTORY config documentation launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL)
install(PROGRAMS scripts/min_turning_radius_calculator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  OPTIONAL)
