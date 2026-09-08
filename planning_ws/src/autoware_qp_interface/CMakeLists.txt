cmake_minimum_required(VERSION 3.0.2)
project(autoware_qp_interface)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED CONFIG)
find_package(proxsuite 0.6.5 REQUIRED CONFIG)
get_target_property(EIGEN3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(OSQP_TARGET_INCLUDE_DIRS osqp::osqp INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(PROXSUITE_INCLUDE_DIRS proxsuite::proxsuite INTERFACE_INCLUDE_DIRECTORIES)
set(OSQP_INCLUDE_DIRS "")
foreach(_osqp_target_include IN LISTS OSQP_TARGET_INCLUDE_DIRS)
  if(EXISTS "${_osqp_target_include}/osqp.h")
    get_filename_component(_osqp_include_parent "${_osqp_target_include}" DIRECTORY)
    list(APPEND OSQP_INCLUDE_DIRS "${_osqp_include_parent}")
  else()
    list(APPEND OSQP_INCLUDE_DIRS "${_osqp_target_include}")
  endif()
endforeach()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CFG_EXTRAS autoware_qp_interface-extras.cmake
)

add_library(${PROJECT_NAME} SHARED
  src/qp_interface.cpp
  src/osqp_interface.cpp
  src/osqp_csc_matrix_conv.cpp
  src/proxqp_interface.cpp
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-error=old-style-cast)
target_include_directories(${PROJECT_NAME}
  PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${OSQP_INCLUDE_DIRS} ${PROXSUITE_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC osqp::osqp proxsuite::proxsuite)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

# Replace catkin's absolute-prefix .pc with an install-tree-relocatable form.
# External provider includes/libs are intentionally supplied by Config extras,
# never serialized into pkg-config.
install(CODE "file(WRITE \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/pkgconfig/autoware_qp_interface.pc\" \"prefix=\\\${pcfiledir}/../..\\n\\nName: autoware_qp_interface\\nDescription: Interface for the QP solvers\\nVersion: 1.8.0\\nCflags: -I\\\${prefix}/include\\nLibs: -L\\\${prefix}/lib -lautoware_qp_interface\\n\")")

# Catkin generates its install-space Config.cmake late in configure, so apply
# the narrow prefix rewrite after that file has actually been installed.
install(CODE [=[
  set(_qpi_config "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/autoware_qp_interface/cmake/autoware_qp_interfaceConfig.cmake")
  file(READ "${_qpi_config}" _qpi_text)
  set(_qpi_install_literal "\${autoware_qp_interface_INSTALL_PREFIX}")
  string(REPLACE "${CMAKE_INSTALL_PREFIX}" "${_qpi_install_literal}" _qpi_text "${_qpi_text}")
  string(REGEX REPLACE
    [[set\(autoware_qp_interface_SOURCE_PREFIX [^\)]*\)]]
    [[set(autoware_qp_interface_SOURCE_PREFIX "")]] _qpi_text "${_qpi_text}")
  string(REGEX REPLACE
    [[set\(autoware_qp_interface_DEVEL_PREFIX [^\)]*\)]]
    [[set(autoware_qp_interface_DEVEL_PREFIX "")]] _qpi_text "${_qpi_text}")
  set(_qpi_install_replacement
    "get_filename_component(autoware_qp_interface_INSTALL_PREFIX \"\${autoware_qp_interface_DIR}/../../..\" ABSOLUTE)")
  string(REGEX REPLACE
    [[set\(autoware_qp_interface_INSTALL_PREFIX [^\)]*\)]]
    "${_qpi_install_replacement}" _qpi_text "${_qpi_text}")
  file(WRITE "${_qpi_config}" "${_qpi_text}")
]=])

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_osqp_interface.cpp
    test/test_csc_matrix_conv.cpp
    test/test_proxqp_interface.cpp
    test/test_qp_interface.cpp)
  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} gtest_main)
  endif()
endif()
