cmake_minimum_required(VERSION 3.0.2)
project(autoware_map_msgs)

find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(FILES
  AreaInfo.msg
  LaneletMapBin.msg
  LaneletMapCellMetaData.msg
  LaneletMapMetaData.msg
  MapProjectorInfo.msg
  PointCloudMapCellMetaData.msg
  PointCloudMapCellMetaDataWithID.msg
  PointCloudMapCellWithID.msg
  PointCloudMapMetaData.msg
)

add_service_files(FILES
  GetDifferentialPointCloudMap.srv
  GetPartialPointCloudMap.srv
  GetSelectedLanelet2Map.srv
  GetSelectedPointCloudMap.srv
)

generate_messages(DEPENDENCIES geographic_msgs geometry_msgs sensor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS geographic_msgs geometry_msgs message_runtime sensor_msgs std_msgs)
