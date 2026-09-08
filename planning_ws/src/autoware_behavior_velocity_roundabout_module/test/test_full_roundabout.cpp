#include <autoware/behavior_velocity_roundabout_module/ros1_support.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>
#include "object_manager.hpp"
#include "decision_result.hpp"
#include <pluginlib/class_loader.hpp>
#include <gtest/gtest.h>
#include <cstdlib>
namespace bvp = autoware::behavior_velocity_planner;
template<class Interface> void plugins(const std::string & base, const std::string & prefix)
{
  ros::NodeHandle node("/full_roundabout_test");
  ASSERT_EQ(std::system("rosparam load \"$(rospack find autoware_behavior_velocity_planner)/config/ros1/behavior_velocity_planner.yaml\" /full_roundabout_test"), 0);
  ASSERT_EQ(std::system("rosparam load \"$(rospack find autoware_behavior_velocity_intersection_module)/config/ros1/intersection.yaml\" /full_roundabout_test"), 0);
  ASSERT_EQ(std::system("rosparam load \"$(rospack find autoware_behavior_velocity_roundabout_module)/config/ros1/roundabout.yaml\" /full_roundabout_test"), 0);
  pluginlib::ClassLoader<Interface> loader("autoware_behavior_velocity_planner_common", base);
  // Hold both libraries and both plugin objects in the same process/lifetime.
  auto intersection = loader.createUniqueInstance(prefix + "IntersectionModulePlugin");
  auto roundabout = loader.createUniqueInstance(prefix + "RoundaboutModulePlugin");
  intersection->init(node);
  roundabout->init(node);
  EXPECT_STREQ(roundabout->getModuleName(), "roundabout");
  auto required = roundabout->getRequiredSubscriptions();
  EXPECT_TRUE(required.predicted_objects);
  EXPECT_FALSE(required.traffic_signals);
  EXPECT_FALSE(required.occupancy_grid_map);
  bvp::RoundaboutObjectInfoManager manager;
  unique_identifier_msgs::UUID uuid; uuid.uuid[0] = 42;
  auto object = manager.registerObject(uuid, true);
  ASSERT_TRUE(object);
  autoware_perception_msgs::PredictedObject prediction;
  prediction.object_id = uuid;
  prediction.kinematics.initial_twist_with_covariance.twist.linear.x = 3.0;
  object->initialize(prediction, std::nullopt);
  EXPECT_DOUBLE_EQ(object->observed_velocity(), 3.0);
  EXPECT_FALSE(object->estimated_past_position(1.0));
  bvp::RoundaboutCollisionInterval interval;
  interval.interval_time = {1.0, 2.0};
  object->update_safety(interval, std::nullopt);
  ASSERT_TRUE(object->unsafe_info());
  EXPECT_EQ(object->unsafe_info()->interval_time, std::make_pair(1.0, 2.0));
  EXPECT_EQ(manager.allObjects().size(), 1U);
  EXPECT_EQ(manager.registerObject(uuid, false).get(), object.get());
  object->initialize(prediction, std::nullopt);
  EXPECT_FALSE(object->unsafe_info());
  manager.clearObjects();
  EXPECT_TRUE(manager.allObjects().empty());
  EXPECT_TRUE(manager.getObjectsMap().empty());
}
TEST(FullRoundabout, NormalCoLoadsIntersectionAndPreservesObjectState)
{
  plugins<bvp::PluginInterface>("autoware::behavior_velocity_planner::PluginInterface",
    "autoware::behavior_velocity_planner::");
}
TEST(FullRoundabout, ExperimentalCoLoadsIntersectionAndPreservesObjectState)
{
  plugins<bvp::experimental::PluginInterface>("autoware::behavior_velocity_planner::experimental::PluginInterface",
    "autoware::behavior_velocity_planner::experimental::");
}
TEST(FullRoundabout, OriginalGeometricPassageUsesDurationAndRejectsDistantPolygon)
{
  autoware_perception_msgs::PredictedPath path;
  path.time_step.sec = 0; path.time_step.nanosec = 500000000;
  for (int x = -3; x <= 3; ++x) {
    geometry_msgs::Pose pose; pose.position.x = x; pose.orientation.w = 1.0;
    path.path.push_back(pose);
  }
  autoware_perception_msgs::Shape shape;
  shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  shape.dimensions.x = 1; shape.dimensions.y = 1; shape.dimensions.z = 1;
  lanelet::BasicPolygon2d polygon{{-0.5, -1}, {0.5, -1}, {0.5, 1}, {-0.5, 1}};
  const auto interval = bvp::findRoundaboutPassageInterval(path, shape, polygon, std::nullopt);
  ASSERT_TRUE(interval);
  EXPECT_EQ(interval->interval_position, std::make_pair(size_t{1}, size_t{5}));
  EXPECT_EQ(interval->interval_time, std::make_pair(0.5, 2.5));
  for (auto & point : polygon) point.x() += 100;
  EXPECT_FALSE(bvp::findRoundaboutPassageInterval(path, shape, polygon, std::nullopt));
}
TEST(FullRoundabout, OriginalDecisionDiagnosticsPreserveRtcAndError)
{
  EXPECT_EQ(bvp::formatDecisionResult(bvp::RoundaboutInternalError{"missing lane"}, false),
    "RTC: roundabout activated_ = 0\nInternalError because missing lane");
  EXPECT_EQ(bvp::formatDecisionResult(bvp::RoundaboutCollisionStop{}, true),
    "RTC: roundabout activated_ = 1\nCollisionStop:\n");
}
int main(int argc, char ** argv) {
  ros::init(argc, argv, "full_roundabout_tests");
  testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS();
}
