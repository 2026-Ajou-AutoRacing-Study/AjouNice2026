cmake_minimum_required(VERSION 3.0.2)
project(trajectory_validator)

find_package(catkin REQUIRED COMPONENTS
  autoware_internal_planning_msgs
  autoware_planning_msgs
  geometry_msgs
  message_generation
  morai_msgs
  nav_msgs
  pluginlib
  roscpp
  rospy
  rviz
  std_msgs
  std_srvs
  tier4_planning_msgs
  trajectory_follower
  visualization_msgs
)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)

catkin_python_setup()

add_message_files(FILES TrajectoryDebug.msg)

generate_messages(DEPENDENCIES
  autoware_planning_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  LIBRARIES trajectory_validator_rviz_panel
  CATKIN_DEPENDS
    autoware_internal_planning_msgs
    autoware_planning_msgs
    geometry_msgs
    message_runtime
    morai_msgs
    nav_msgs
    pluginlib
    roscpp
    rospy
    rviz
    std_msgs
    std_srvs
    tier4_planning_msgs
    trajectory_follower
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(trajectory_validator_rviz_panel
  src/planner_csv_logger_panel.cpp
  src/planning_status_panel.cpp
  src/purepursuit_hud_panel.cpp
  include/trajectory_validator/planner_csv_logger_panel.hpp
  include/trajectory_validator/planning_status_panel.hpp
  include/trajectory_validator/purepursuit_hud_panel.hpp
)
add_dependencies(trajectory_validator_rviz_panel
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(trajectory_validator_rviz_panel
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_csv_io.py)
  catkin_add_nosetests(test/test_history_v5.py)
  catkin_add_nosetests(test/test_velocity_units.py)
  catkin_add_nosetests(test/test_goal_guard.py)
endif()

catkin_install_python(PROGRAMS
  scripts/trajectory_validator_node.py
  scripts/publish_history.py
  scripts/publish_final_path.py
  scripts/publish_recent_ego_paths.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS trajectory_validator_rviz_panel
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config launch msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
