cmake_minimum_required(VERSION 3.0.2)
project(tf_pub)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  morai_msgs
  roscpp
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs morai_msgs roscpp tf2 tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ego_tf_broadcaster src/ego_tf_broadcaster.cpp)
add_executable(ego_tf_broadcaster_deg_correction src/ego_tf_broadcaster_deg_correction.cpp)
add_executable(gps_tf_broadcaster src/gps_tf_broadcaster.cpp)
add_dependencies(ego_tf_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ego_tf_broadcaster_deg_correction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(gps_tf_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ego_tf_broadcaster
  ${catkin_LIBRARIES}
)
target_link_libraries(ego_tf_broadcaster_deg_correction
  ${catkin_LIBRARIES}
)
target_link_libraries(gps_tf_broadcaster
  ${catkin_LIBRARIES}
)

install(TARGETS ego_tf_broadcaster ego_tf_broadcaster_deg_correction gps_tf_broadcaster
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
