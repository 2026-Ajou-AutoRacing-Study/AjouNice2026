cmake_minimum_required(VERSION 3.0.2)
project(autoware_rtc_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tier4_rtc_msgs
  unique_identifier_msgs
)
find_package(rostest REQUIRED)
find_package(autoware_utils REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES autoware_rtc_interface
  CATKIN_DEPENDS roscpp tier4_rtc_msgs unique_identifier_msgs
  CFG_EXTRAS autoware_rtc_interface-cxx17-extras.cmake
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(autoware_rtc_interface
  src/rtc_interface.cpp
)
add_dependencies(autoware_rtc_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(autoware_rtc_interface ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
  test/test_rtc_interface.cpp
  )
  if(TARGET test_${PROJECT_NAME})
    add_dependencies(test_${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
    target_include_directories(test_${PROJECT_NAME} PRIVATE ${autoware_utils_INCLUDE_DIRS})
    target_link_libraries(test_${PROJECT_NAME}
      autoware_rtc_interface
      ${catkin_LIBRARIES}
      ${autoware_utils_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_${PROJECT_NAME}_owner_contracts test/owner_contracts.cpp)
  if(TARGET test_${PROJECT_NAME}_owner_contracts)
    add_dependencies(test_${PROJECT_NAME}_owner_contracts ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test_${PROJECT_NAME}_owner_contracts autoware_rtc_interface ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS autoware_rtc_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
