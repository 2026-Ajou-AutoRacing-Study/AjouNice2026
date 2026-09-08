#include <autoware/planning_validator_intersection_collision_checker/object_tracking_core_ros1.hpp>

#include <gtest/gtest.h>

namespace icc = autoware::planning_validator::intersection_collision_checker_ros1;

TEST(ObjectTrackingCoreRos1, EstimatesStableApproachVelocity)
{
  icc::ObjectTrackingCore core(ros::Time(10.0), 10.0);
  core.update(ros::Time(10.2), 9.8, 25.0, 20.0, 3.0);
  core.update(ros::Time(10.4), 9.6, 25.0, 20.0, 3.0);
  EXPECT_NEAR(core.velocity(), 1.0, 1e-6);
  EXPECT_NEAR(core.track_duration(), 0.4, 1e-6);
  EXPECT_EQ(core.history_size(), 3U);
}

TEST(ObjectTrackingCoreRos1, RejectsRawVelocityOutlierBySourceResetRule)
{
  icc::ObjectTrackingCore core(ros::Time(10.0), 10.0);
  core.update(ros::Time(10.2), 1.0, 25.0, 20.0, 3.0);
  EXPECT_EQ(core.history_size(), 1U);
  EXPECT_DOUBLE_EQ(core.track_duration(), 0.0);
  EXPECT_DOUBLE_EQ(core.velocity(), 0.0);
  EXPECT_NEAR(core.distance_to_overlap(), 1.0, 1e-12);
}
