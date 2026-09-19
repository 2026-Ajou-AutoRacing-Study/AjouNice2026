cmake_minimum_required(VERSION 3.0.2)
project(autoware_osqp_interface)

find_package(catkin REQUIRED COMPONENTS roscpp)

find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED CONFIG)
get_target_property(OSQP_INCLUDE_SUB_DIR osqp::osqp INTERFACE_INCLUDE_DIRECTORIES)
get_filename_component(OSQP_INCLUDE_DIR ${OSQP_INCLUDE_SUB_DIR} PATH)
get_target_property(OSQP_LIBRARY_FILE osqp::osqp IMPORTED_LOCATION_RELEASE)
if(NOT OSQP_LIBRARY_FILE)
  get_target_property(OSQP_LIBRARY_FILE osqp::osqp IMPORTED_LOCATION)
endif()
if(NOT OSQP_LIBRARY_FILE)
  message(FATAL_ERROR "osqp::osqp must expose an imported shared-library location")
endif()
get_filename_component(OSQP_LIBRARY_DIR "${OSQP_LIBRARY_FILE}" DIRECTORY)
set(osqp_INCLUDE_DIRS ${OSQP_INCLUDE_DIR})
set(osqp_LIBRARIES osqp::osqp)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

set(OSQP_INTERFACE_LIB_SRC
  src/csc_matrix_conv.cpp
  src/osqp_interface.cpp
)

set(OSQP_INTERFACE_LIB_HEADERS
  include/autoware/osqp_interface/csc_matrix_conv.hpp
  include/autoware/osqp_interface/osqp_interface.hpp
  include/autoware/osqp_interface/visibility_control.hpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS Eigen3 osqp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OSQP_INCLUDE_DIR})

add_library(${PROJECT_NAME} SHARED
  ${OSQP_INTERFACE_LIB_SRC}
  ${OSQP_INTERFACE_LIB_HEADERS}
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-error=old-style-cast)

target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC
    "${EIGEN3_INCLUDE_DIRS}"
    "${OSQP_INCLUDE_DIR}"
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} osqp::osqp)

# The approved OSQP provider is a dedicated prefix, not a system library.  A
# sourced producer install must therefore expose its loader directory instead
# of relying on an interactive LD_LIBRARY_PATH override.
catkin_add_env_hooks(99.osqp_provider
  SHELLS sh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks
)

if(CATKIN_ENABLE_TESTING)
  set(TEST_SOURCES
    test/test_osqp_interface.cpp
    test/test_csc_matrix_conv.cpp
  )
  set(TEST_OSQP_INTERFACE_EXE test_osqp_interface)
  catkin_add_gtest(${TEST_OSQP_INTERFACE_EXE} ${TEST_SOURCES})
  if(TARGET ${TEST_OSQP_INTERFACE_EXE})
    target_link_libraries(${TEST_OSQP_INTERFACE_EXE} ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)
  endif()
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
