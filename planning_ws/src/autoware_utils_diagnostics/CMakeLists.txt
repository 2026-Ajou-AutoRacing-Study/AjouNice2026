cmake_minimum_required(VERSION 3.0.2)
project(autoware_utils_diagnostics)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  diagnostic_updater
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS diagnostic_msgs diagnostic_updater roscpp
  CFG_EXTRAS autoware_utils_diagnostics-extras.cmake
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/timeout_diagnostics.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/main.cpp
    test/cases/diagnostics_interface.cpp
    test/cases/timeout_diagnostics.cpp
  )
  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME}
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES cmake/autoware_utils_diagnostics-extras.cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
)
