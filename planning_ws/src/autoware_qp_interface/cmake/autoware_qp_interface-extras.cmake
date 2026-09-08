# Rediscover external providers at consumer configure time.
# No build-time provider prefix is serialized into this installed file.
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED CONFIG)
find_package(proxsuite 0.6.5 REQUIRED CONFIG)

get_target_property(_autoware_qpi_eigen_includes Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(_autoware_qpi_osqp_target_includes osqp::osqp INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(_autoware_qpi_proxsuite_includes proxsuite::proxsuite INTERFACE_INCLUDE_DIRECTORIES)
set(_autoware_qpi_osqp_includes "")
foreach(_autoware_qpi_osqp_target_include IN LISTS _autoware_qpi_osqp_target_includes)
  if(EXISTS "${_autoware_qpi_osqp_target_include}/osqp.h")
    get_filename_component(_autoware_qpi_osqp_include_parent
      "${_autoware_qpi_osqp_target_include}" DIRECTORY)
    list(APPEND _autoware_qpi_osqp_includes "${_autoware_qpi_osqp_include_parent}")
  else()
    list(APPEND _autoware_qpi_osqp_includes "${_autoware_qpi_osqp_target_include}")
  endif()
endforeach()

list(APPEND autoware_qp_interface_INCLUDE_DIRS
  ${_autoware_qpi_eigen_includes}
  ${_autoware_qpi_osqp_includes}
  ${_autoware_qpi_proxsuite_includes})
list(APPEND autoware_qp_interface_LIBRARIES osqp::osqp proxsuite::proxsuite)
