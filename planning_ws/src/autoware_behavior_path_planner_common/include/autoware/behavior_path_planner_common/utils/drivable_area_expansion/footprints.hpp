#pragma once

#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/parameters.hpp>
#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/types.hpp>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <geometry_msgs/Pose.h>

namespace autoware::behavior_path_planner::drivable_area_expansion
{
Polygon2d translate_polygon(const Polygon2d & polygon, double x, double y);
Polygon2d create_footprint(const geometry_msgs::Pose & pose, const Polygon2d & base_footprint);
MultiPolygon2d create_object_footprints(const autoware_perception_msgs::PredictedObjects & objects,
  const DrivableAreaExpansionParameters & params);
}
