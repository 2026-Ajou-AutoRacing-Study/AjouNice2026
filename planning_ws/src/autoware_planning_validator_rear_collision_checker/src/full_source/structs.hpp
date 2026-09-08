// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef STRUCTS_HPP_
#define STRUCTS_HPP_

#include <autoware/planning_validator_rear_collision_checker/full_source/rear_collision_checker_node_parameters.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_internal_debug_msgs/StringStamped.h>
#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_validator::full_source
{

using autoware::vehicle_info_utils::VehicleInfo;
using autoware_internal_debug_msgs::StringStamped;
using autoware_map_msgs::LaneletMapBin;
using autoware_planning_msgs::LaneletRoute;
using autoware_planning_msgs::Trajectory;
using autoware_planning_msgs::TrajectoryPoint;

using geometry_msgs::AccelWithCovarianceStamped;
using nav_msgs::Odometry;
using sensor_msgs::PointCloud2;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

using DetectionArea = std::pair<lanelet::BasicPolygon3d, lanelet::ConstLanelets>;

using DetectionAreas = std::vector<DetectionArea>;

enum class Behavior {
  NONE = 0,
  SHIFT_LEFT,
  SHIFT_RIGHT,
  TURN_LEFT,
  TURN_RIGHT,
};

struct PointCloudObject
{
  ros::Time last_update_time;

  ros::Time last_stop_time;

  geometry_msgs::Pose pose;

  lanelet::ConstLanelet furthest_lane;

  double tracking_duration{0.0};

  double absolute_distance{0.0};

  double relative_distance{0.0};

  double relative_distance_with_delay_compensation{0.0};

  double time_to_collision{0.0};

  double rss_distance{0.0};

  double velocity{0.0};

  double moving_time{0.0};

  bool safe{true};

  bool ignore{false};

  bool is_vru{false};

  std::string detail{""};
};

using PointCloudObjects = std::vector<PointCloudObject>;

struct RearDebugData
{
  autoware_utils_geometry::LineString3d reachable_line;

  autoware_utils_geometry::LineString3d stoppable_line;

  lanelet::ConstLanelets current_lanes;

  PointCloudObjects pointcloud_objects;

  DetectionAreas detection_areas;

  sensor_msgs::PointCloud2::Ptr cluster_points;

  sensor_msgs::PointCloud2::Ptr voxel_points;

  Behavior turn_behavior{Behavior::NONE};

  Behavior shift_behavior{Behavior::NONE};

  std::vector<autoware_utils_geometry::Polygon3d> hull_polygons;

  std::vector<size_t> pointcloud_nums{};

  std::string text{"-"};

  double processing_time_detail_ms;

  bool is_active{false};

  bool is_safe{true};

  auto get_detection_polygons() const -> lanelet::BasicPolygons3d
  {
    lanelet::BasicPolygons3d ret{};
    for (const auto & [polygon, lanes] : detection_areas) {
      ret.push_back(polygon);
    }
    return ret;
  }

  auto get_detection_lanes() const -> lanelet::ConstLanelets
  {
    lanelet::ConstLanelets ret{};
    for (const auto & [polygon, lanes] : detection_areas) {
      ret.insert(ret.end(), lanes.begin(), lanes.end());
    }
    return ret;
  }
};
}  // namespace autoware::planning_validator::full_source

#endif  // STRUCTS_HPP_
