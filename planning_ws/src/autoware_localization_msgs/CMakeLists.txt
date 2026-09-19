cmake_minimum_required(VERSION 3.0.2)
project(autoware_localization_msgs)

find_package(catkin REQUIRED COMPONENTS
  autoware_common_msgs
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(FILES
  KinematicState.msg
)

add_service_files(FILES
  InitializeLocalization.srv
)

generate_messages(DEPENDENCIES
  autoware_common_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  autoware_common_msgs
  geometry_msgs
  message_runtime
  std_msgs
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
