// Copyright 2026 AjouNice2026
// Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE_PATH_OPTIMIZER_GAUSSIAN_RESAMPLING_HPP_
#define AUTOWARE_PATH_OPTIMIZER_GAUSSIAN_RESAMPLING_HPP_

#include "autoware/path_optimizer/type_alias.hpp"
#include <vector>

namespace autoware::path_optimizer::trajectory_utils
{
// Spatial sigma in metres, independent of MPT/output point spacing. Uses a
// 0.1m Akima support grid, Gaussian XY smoothing and cubic output resampling.
// The first/last metre and +/-1m around ego retain their original geometry;
// smoothing fades in over the next 2m. Velocity, Z and stop stations are kept.
// Callers must validate the FINAL footprint, road bounds and curvature.
std::vector<TrajectoryPoint> resampleTrajectoryPointsGaussian(
  const std::vector<TrajectoryPoint> & points, double output_interval_m,
  double sigma_m, const geometry_msgs::Pose & ego_pose);
}
#endif
