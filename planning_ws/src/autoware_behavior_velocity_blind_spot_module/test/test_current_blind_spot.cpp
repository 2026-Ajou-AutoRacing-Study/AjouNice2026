#include <autoware/behavior_velocity_blind_spot_module/ros1_support.hpp>
#include <autoware/behavior_velocity_blind_spot_module/util.hpp>
#include <autoware/behavior_velocity_blind_spot_module/time_to_collision.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <gtest/gtest.h>
namespace bvp = autoware::behavior_velocity_planner;
template<class Interface> void plugins(const std::string & base, const std::string & prefix)
{
  ros::NodeHandle node("/full_blind_spot_test");
  pluginlib::ClassLoader<Interface> loader("autoware_behavior_velocity_planner_common", base);
  auto intersection = loader.createUniqueInstance(prefix + "IntersectionModulePlugin");
  auto roundabout = loader.createUniqueInstance(prefix + "RoundaboutModulePlugin");
  auto blind = loader.createUniqueInstance(prefix + "BlindSpotModulePlugin");
  intersection->init(node); roundabout->init(node); blind->init(node);
  EXPECT_STREQ(blind->getModuleName(), "blind_spot");
  const auto required = blind->getRequiredSubscriptions();
  EXPECT_TRUE(required.predicted_objects);
  EXPECT_FALSE(required.traffic_signals);
  EXPECT_FALSE(required.occupancy_grid_map);
}
TEST(CurrentBlindSpot, NormalPluginsCoexist)
{
  plugins<bvp::PluginInterface>("autoware::behavior_velocity_planner::PluginInterface",
    "autoware::behavior_velocity_planner::");
}
TEST(CurrentBlindSpot, ExperimentalPluginsCoexist)
{
  plugins<bvp::experimental::PluginInterface>("autoware::behavior_velocity_planner::experimental::PluginInterface",
    "autoware::behavior_velocity_planner::experimental::");
}
TEST(CurrentBlindSpot, CurrentInterpolationAndLaneIdContract)
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  for (int i=0; i<=20; ++i) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = i;
    point.point.pose.orientation.w = 1;
    point.point.longitudinal_velocity_mps = 4;
    point.lane_ids = {i < 5 ? 10 : (i < 15 ? 42 : 90)};
    path.points.push_back(point);
  }
  EXPECT_EQ(bvp::find_lane_ids_upto(path, 42), std::vector<lanelet::Id>{10});
  const auto result = bvp::generateInterpolatedPathInfo(42, path, "test");
  ASSERT_TRUE(result);
  EXPECT_DOUBLE_EQ(result->ds, 0.2);
  EXPECT_EQ(result->lane_id, 42);
  EXPECT_LT(result->lane_id_interval.first, result->lane_id_interval.second);
  EXPECT_GT(result->path.points.size(), path.points.size());
  EXPECT_FALSE(bvp::generateInterpolatedPathInfo(123456, path, "test"));
  EXPECT_FALSE(bvp::generateInterpolatedPathInfo(42, {}, "test"));
}
TEST(CurrentBlindSpot, OriginalTtcOverlapMarginsAndClearedObject)
{
  EXPECT_FALSE(bvp::get_unsafe_time_if_critical({5, 6}, {1, 2}, 0.5, 0.5));
  const auto overlap = bvp::get_unsafe_time_if_critical({2, 5}, {3, 4}, 0.5, 0.5);
  ASSERT_TRUE(overlap); EXPECT_DOUBLE_EQ(*overlap, 3);
  const auto delayed = bvp::get_unsafe_time_if_critical({2, 5}, {0, 1.6}, 0.5, 0.5);
  ASSERT_TRUE(delayed); EXPECT_DOUBLE_EQ(*delayed, 0);
}
TEST(CurrentBlindSpot, OriginalFootprintEntryExitTimes)
{
  std::vector<bvp::FuturePosition> positions;
  for (int i=0; i<=10; ++i) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = i; point.point.pose.orientation.w = 1;
    positions.push_back({point, double(i)});
  }
  autoware_utils_geometry::LinearRing2d footprint{{-0.4,-0.4},{0.4,-0.4},{0.4,0.4},{-0.4,0.4},{-0.4,-0.4}};
  lanelet::LineString3d entry{100, {lanelet::Point3d{101,2,-2,0}, lanelet::Point3d{102,2,2,0}}};
  lanelet::LineString3d exit{200, {lanelet::Point3d{201,7,-2,0}, lanelet::Point3d{202,7,2,0}}};
  const auto interval = bvp::compute_time_interval_for_passing_line(positions, footprint, entry, exit);
  ASSERT_TRUE(interval); EXPECT_EQ(*interval, std::make_pair(2.0, 7.0));
  EXPECT_FALSE(bvp::compute_time_interval_for_passing_line(std::vector<bvp::FuturePosition>{}, footprint, entry, exit));
}
int main(int argc, char ** argv) {
  ros::init(argc, argv, "full_blind_spot_current_tests");
  testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS();
}
