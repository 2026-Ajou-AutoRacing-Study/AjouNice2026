cmake_minimum_required(VERSION 3.14)
project(autoware_vehicle_info_utils)

find_package(catkin REQUIRED COMPONENTS roscpp autoware_utils_geometry rostest)
find_package(Eigen3 REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vehicle_info_utils
  CATKIN_DEPENDS roscpp autoware_utils_geometry
  DEPENDS EIGEN3
)

add_library(vehicle_info_utils
  src/vehicle_info.cpp
  src/vehicle_info_utils.cpp
)
target_include_directories(vehicle_info_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CATKIN_PACKAGE_INCLUDE_DESTINATION}>
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(vehicle_info_utils ${catkin_LIBRARIES} Eigen3::Eigen)

install(TARGETS vehicle_info_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_rostest_gtest(test_${PROJECT_NAME}
    test/vehicle_info.test
    test/main.cpp
    test/test_vehicle_info_utils.cpp
  )
  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME} vehicle_info_utils ${catkin_LIBRARIES})
  endif()
endif()
