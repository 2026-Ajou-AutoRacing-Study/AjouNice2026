cmake_minimum_required(VERSION 3.0.2)
project(autoware_universe_utils)

find_package(catkin REQUIRED COMPONENTS
  autoware_utils
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS autoware_utils
)

add_library(${PROJECT_NAME}
  src/geometry/polygon_clip.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(universe_utils_header_oracle test/header_oracle.cpp)
  if(TARGET universe_utils_header_oracle)
    target_include_directories(universe_utils_header_oracle PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(universe_utils_header_oracle ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)
  endif()
endif()

install(DIRECTORY include/autoware/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
