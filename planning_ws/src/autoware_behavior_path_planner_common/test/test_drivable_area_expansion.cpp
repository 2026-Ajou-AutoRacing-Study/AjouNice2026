#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/drivable_area_expansion.hpp>
#include <gtest/gtest.h>

namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;

TEST(DrivableAreaExpansionSourceSurface, PreservesCoreAlgorithms)
{
  expansion::DrivableAreaExpansionParameters params;
  params.vehicle_info.front_overhang_m = 1.0;
  params.vehicle_info.vehicle_width_m = 2.0;
  params.vehicle_info.wheel_base_m = 3.0;
  EXPECT_GT(expansion::calculate_minimum_lane_width(10.0, params), 0.0);
  std::vector<geometry_msgs::Pose> poses(3);
  poses.at(0).orientation.w = poses.at(1).orientation.w = poses.at(2).orientation.w = 1.0;
  EXPECT_EQ(expansion::calculate_smoothed_curvatures(poses, 3).size(), poses.size());
}
