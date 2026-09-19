#include <autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp>
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/safety_core.hpp>

#include <gtest/gtest.h>

namespace safety = autoware::behavior_path_planner::utils::path_safety_checker;

TEST(PathSafetyParameterSurface, PreservesSourceDefaultsAndCoreInteroperability)
{
  safety::SafetyCheckParams safety_params;
  EXPECT_FALSE(safety_params.enable_safety_check);
  EXPECT_EQ(safety_params.method, "RSS");
  EXPECT_DOUBLE_EQ(safety_params.collision_check_yaw_diff_threshold, 3.1416);
  safety::ObjectsFilteringParams filtering;
  filtering.object_check_forward_distance = 50.0;
  filtering.use_all_predicted_path = true;
  EXPECT_DOUBLE_EQ(filtering.object_check_forward_distance, 50.0);
  EXPECT_TRUE(filtering.use_all_predicted_path);
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  safety::PoseWithVelocityStamped stamped{1.5, pose, 2.0};
  EXPECT_DOUBLE_EQ(stamped.time, 1.5);
  EXPECT_DOUBLE_EQ(stamped.velocity, 2.0);
  EXPECT_DOUBLE_EQ(safety::calc_rss_distance(4.0, 6.0, safety_params.rss_params), 10.0);
}
