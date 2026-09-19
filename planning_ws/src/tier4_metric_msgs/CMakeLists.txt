cmake_minimum_required(VERSION 3.0.2)
project(tier4_metric_msgs)

find_package(catkin REQUIRED COMPONENTS
  builtin_interfaces
  message_generation
)

add_message_files(FILES
  Metric.msg
  MetricArray.msg
)

generate_messages(DEPENDENCIES
  builtin_interfaces
)

catkin_package(CATKIN_DEPENDS
  builtin_interfaces
  message_runtime
)
