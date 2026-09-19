// Copyright 2022 TIER IV, Inc. Licensed under Apache-2.0.
// Original marker_utils footprint/predicted-path implementations, ROS1 clock API.
#include <autoware/behavior_path_planner_common/marker_utils/utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

namespace marker_utils {
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_scale;
using autoware_utils::create_marker_color;

void addFootprintMarker(Marker & marker, const geometry_msgs::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & info)
{
  const double half_width = info.vehicle_width_m / 2.0;
  const double front = info.vehicle_length_m - info.rear_overhang_m;
  const double rear = info.rear_overhang_m;
  marker.points.push_back(autoware_utils::calc_offset_pose(pose, front, -half_width, 0.0).position);
  marker.points.push_back(autoware_utils::calc_offset_pose(pose, front, half_width, 0.0).position);
  marker.points.push_back(autoware_utils::calc_offset_pose(pose, -rear, half_width, 0.0).position);
  marker.points.push_back(autoware_utils::calc_offset_pose(pose, -rear, -half_width, 0.0).position);
  marker.points.push_back(marker.points.front());
}

MarkerArray createFootprintMarkerArray(const geometry_msgs::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo info, const std::string && ns,
  const int32_t & id, const float & r, const float & g, const float & b)
{
  auto marker = create_default_marker("map", ros::Time::now(), ns, id, Marker::LINE_STRIP,
    create_marker_scale(0.2, 0.2, 0.2), create_marker_color(r, g, b, 0.999));
  marker.lifetime = ros::Duration(1.5);
  addFootprintMarker(marker, pose, info);
  MarkerArray output;
  output.markers.push_back(marker);
  return output;
}

MarkerArray createPredictedPathMarkerArray(const autoware_perception_msgs::PredictedPath & path,
  const autoware::vehicle_info_utils::VehicleInfo & info, std::string && ns,
  const int32_t & id, const float & r, const float & g, const float & b)
{
  if (path.path.empty()) return {};
  auto marker = create_default_marker("map", ros::Time::now(), ns, id, Marker::LINE_STRIP,
    create_marker_scale(0.1, 0.1, 0.1), create_marker_color(r, g, b, 0.999));
  marker.lifetime = ros::Duration(1.5);
  MarkerArray output;
  for (size_t i = 0; i < path.path.size(); ++i) {
    marker.id = i + id;
    marker.points.clear();
    addFootprintMarker(marker, path.path.at(i), info);
    output.markers.push_back(marker);
  }
  return output;
}
}
