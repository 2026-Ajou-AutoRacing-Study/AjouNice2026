cmake_minimum_required(VERSION 3.0.2)
project(rviz_2d_overlay_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(FILES
  OverlayText.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
)
