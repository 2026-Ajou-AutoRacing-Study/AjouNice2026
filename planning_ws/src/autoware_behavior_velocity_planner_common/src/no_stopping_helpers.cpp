// Copyright 2015-2019 Autoware Foundation
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

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
namespace autoware::behavior_velocity_planner::planning_utils {
using autoware_internal_planning_msgs::PathWithLaneId;
using autoware_utils_geometry::Polygon2d;
using autoware_utils_geometry::Point2d;
namespace bg = boost::geometry;
Polygon2d generatePathPolygon(
  const PathWithLaneId & path, const size_t start_idx, const size_t end_idx, const double width)
{
  Polygon2d ego_area;  // open polygon
  for (size_t i = start_idx; i <= end_idx; ++i) {
    const double yaw = tf2::getYaw(path.points.at(i).point.pose.orientation);
    const double x = path.points.at(i).point.pose.position.x + width * std::sin(yaw);
    const double y = path.points.at(i).point.pose.position.y - width * std::cos(yaw);
    ego_area.outer().push_back(Point2d(x, y));
  }
  for (size_t i = end_idx; ; --i) {
    const double yaw = tf2::getYaw(path.points.at(i).point.pose.orientation);
    const double x = path.points.at(i).point.pose.position.x - width * std::sin(yaw);
    const double y = path.points.at(i).point.pose.position.y + width * std::cos(yaw);
    ego_area.outer().push_back(Point2d(x, y));
    if (i == start_idx) break;
  }

  bg::correct(ego_area);
  return ego_area;
}

bool isOverLine(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const geometry_msgs::Pose & self_pose, const geometry_msgs::Pose & line_pose,
  const double offset)
{
  return autoware::motion_utils::calcSignedArcLength(
           path.points, self_pose.position, line_pose.position) +
           offset <
         0.0;
}

}
