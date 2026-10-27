cmake_minimum_required(VERSION 3.0.2)
project(autoware_path_optimizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp geometry_msgs nav_msgs std_msgs visualization_msgs diagnostic_updater tf2 tf2_geometry_msgs
  autoware_internal_debug_msgs autoware_interpolation autoware_motion_utils autoware_osqp_interface
  autoware_planning_msgs autoware_utils autoware_utils_debug autoware_utils_geometry
  autoware_utils_math autoware_utils_rclcpp autoware_utils_system autoware_vehicle_info_utils)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

get_filename_component(AW149_EXPORT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(ACADOS_ROOT "${AW149_EXPORT_ROOT}/external/acados/install" CACHE PATH "export-local acados provider")
if(NOT EXISTS "${ACADOS_ROOT}/lib/libacados.so")
  message(FATAL_ERROR "ACADOS_ROOT must name the staged Path Optimizer provider")
endif()
set(MODEL curvilinear_bicycle_model_spatial)
set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/c_generated_code")
set(GENERATED_MPC_FILES
  ${GEN_DIR}/acados_solver_${MODEL}.c
  ${GEN_DIR}/${MODEL}_model/${MODEL}_expl_ode_fun.c
  ${GEN_DIR}/${MODEL}_model/${MODEL}_expl_vde_adj.c
  ${GEN_DIR}/${MODEL}_model/${MODEL}_expl_vde_forw.c
  ${GEN_DIR}/${MODEL}_constraints/${MODEL}_constr_h_fun.c
  ${GEN_DIR}/${MODEL}_constraints/${MODEL}_constr_h_fun_jac_uxt_zt.c)
add_custom_command(OUTPUT ${GENERATED_MPC_FILES}
  COMMAND ${CMAKE_COMMAND} -E env ACADOS_SOURCE_DIR=${ACADOS_ROOT} PYTHONPATH=${ACADOS_ROOT}/interfaces/acados_template:${CMAKE_CURRENT_SOURCE_DIR}/src/acados_mpc ${ACADOS_ROOT}/venv/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/src/acados_mpc/generators/path_tracking_mpc_spatial_with_body_points.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/acados_mpc/generators/path_tracking_mpc_spatial_with_body_points.py)
add_library(acados_interface SHARED src/acados_mpc/src/acados_interface.cpp ${GENERATED_MPC_FILES})
target_include_directories(acados_interface PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/acados_mpc/include ${CMAKE_CURRENT_BINARY_DIR} ${ACADOS_ROOT}/include ${ACADOS_ROOT}/include/acados ${ACADOS_ROOT}/include/acados_c ${ACADOS_ROOT}/include/blasfeo/include ${ACADOS_ROOT}/include/hpipm/include)
target_link_libraries(acados_interface PUBLIC ${ACADOS_ROOT}/lib/libacados.so)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} acados_interface
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs std_msgs visualization_msgs diagnostic_updater
    autoware_internal_debug_msgs autoware_interpolation autoware_motion_utils autoware_osqp_interface
    autoware_planning_msgs autoware_utils autoware_utils_debug autoware_utils_geometry autoware_utils_math
    autoware_utils_rclcpp autoware_utils_system autoware_vehicle_info_utils)
add_library(${PROJECT_NAME} SHARED
  src/node.cpp src/replan_checker.cpp src/mpt_optimizer.cpp src/obstacle_clustering.cpp src/state_equation_generator.cpp
  src/debug_marker.cpp src/vehicle_model/vehicle_model_interface.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics.cpp src/utils/trajectory_utils.cpp
  src/utils/gaussian_resampling.cpp
  src/utils/geometry_utils.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC include ${CMAKE_CURRENT_SOURCE_DIR}/src/acados_mpc/include ${CMAKE_CURRENT_BINARY_DIR} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC acados_interface ${catkin_LIBRARIES} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} PRIVATE nlohmann_json::nlohmann_json)

add_executable(path_optimizer_node src/main.cpp)
target_compile_features(path_optimizer_node PRIVATE cxx_std_17)
target_link_libraries(path_optimizer_node PRIVATE ${PROJECT_NAME})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_reference_heading_dynamics test/test_reference_heading_dynamics.cpp)
  if(TARGET test_reference_heading_dynamics)
    target_link_libraries(test_reference_heading_dynamics ${PROJECT_NAME})
  endif()
  catkin_add_gtest(test_gaussian_resampling test/test_gaussian_resampling.cpp)
  if(TARGET test_gaussian_resampling)
    target_link_libraries(test_gaussian_resampling ${PROJECT_NAME})
  endif()
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_obstacle_direction_priority test/obstacle_direction_priority.test test/test_obstacle_direction_priority.cpp)
  target_link_libraries(test_obstacle_direction_priority ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_rostest(test/path_extension.test)
  add_rostest_gtest(test_reused_geometry_velocity test/reused_geometry_velocity.test test/test_reused_geometry_velocity.cpp)
  target_link_libraries(test_reused_geometry_velocity ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_rostest_gtest(test_mpt_history_reset test/mpt_history_reset.test test/test_mpt_history_reset.cpp)
  target_link_libraries(test_mpt_history_reset ${PROJECT_NAME} ${catkin_LIBRARIES})
  catkin_add_gtest(test_obstacle_collision test/test_obstacle_collision.cpp)
  if(TARGET test_obstacle_collision)
    target_link_libraries(test_obstacle_collision ${PROJECT_NAME})
  endif()
  catkin_add_gtest(test_mpt_constraint_matrix test/test_mpt_constraint_matrix.cpp)
  if(TARGET test_mpt_constraint_matrix)
    target_link_libraries(test_mpt_constraint_matrix ${PROJECT_NAME})
  endif()
  catkin_add_gtest(test_obstacle_clustering test/test_obstacle_clustering.cpp)
  if(TARGET test_obstacle_clustering)
    target_link_libraries(test_obstacle_clustering ${PROJECT_NAME})
  endif()
endif()

install(TARGETS ${PROJECT_NAME} acados_interface path_optimizer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
