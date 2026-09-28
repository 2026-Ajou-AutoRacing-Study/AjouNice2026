// Copyright 2026 Ajou University

#ifndef AUTOWARE__PATH_OPTIMIZER__OBSTACLE_CLUSTERING_HPP_
#define AUTOWARE__PATH_OPTIMIZER__OBSTACLE_CLUSTERING_HPP_

#include "autoware/path_optimizer/type_alias.hpp"

#include <autoware_planning_msgs/StaticObstacle.h>

#include <cstddef>
#include <vector>

namespace autoware::path_optimizer
{
struct ObstacleClusteringParameters
{
  size_t trigger_count{4};
  double polygon_gap_threshold{0.75};
  double max_longitudinal_span{4.0};
  double max_lateral_span{2.5};
};

std::vector<autoware_planning_msgs::StaticObstacle> clusterStaticObstacles(
  const std::vector<autoware_planning_msgs::StaticObstacle> & obstacles,
  const std::vector<TrajectoryPoint> & reference,
  const ObstacleClusteringParameters & parameters);
}  // namespace autoware::path_optimizer

#endif  // AUTOWARE__PATH_OPTIMIZER__OBSTACLE_CLUSTERING_HPP_
