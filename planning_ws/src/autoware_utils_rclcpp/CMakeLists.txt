cmake_minimum_required(VERSION 3.0.2)
project(autoware_utils_rclcpp)

add_compile_options(-std=c++17)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp std_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_parameter_event test/parameter_event.cpp)
  if(TARGET test_parameter_event)
    target_link_libraries(test_parameter_event ${catkin_LIBRARIES})
  endif()
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_${PROJECT_NAME}
    test/polling.test
    test/main.cpp
    test/cases/parameter.cpp
    test/cases/polling_subscriber.cpp)
  target_link_libraries(test_${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
