cmake_minimum_required(VERSION 3.0.2)
project(autoware_utils_uuid)

find_package(catkin REQUIRED COMPONENTS
  unique_identifier_msgs
)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS unique_identifier_msgs
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/main.cpp
    test/cases/uuid_helper.cpp
  )
  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  endif()
endif()
