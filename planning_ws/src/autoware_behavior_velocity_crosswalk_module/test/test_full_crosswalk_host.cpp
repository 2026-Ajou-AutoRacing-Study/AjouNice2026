// ROS1 transport adaptation of upstream test_planning_factor.cpp.
// Original map, path, odometry, object fixtures and safety assertions retained.
#include <autoware/behavior_velocity_planner/node.hpp>
#include <autoware/behavior_velocity_planner/experimental/node.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>
#include <autoware_internal_planning_msgs/PlanningFactorArray.h>
#include <autoware_planning_msgs/Path.h>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace {
template<class T> void publish(ros::NodeHandle & node, const std::string & name, const T & value)
{
  auto publisher = node.advertise<T>("/full_crosswalk_host/input/" + name, 1, true);
  for (int i=0; i<100 && !publisher.getNumSubscribers(); ++i) {
    ros::spinOnce(); ros::WallDuration(0.01).sleep();
  }
  ASSERT_GT(publisher.getNumSubscribers(), 0u) << name;
  for (int i=0; i<5; ++i) {
    publisher.publish(value); ros::WallDuration(0.04).sleep(); ros::spinOnce();
  }
}
void runCase(bool pedestrian, bool experimental = false)
{
  ros::NodeHandle node;
  node.setParam("/full_crosswalk_host/launch_modules", std::vector<std::string>{experimental ?
    "autoware::behavior_velocity_planner::experimental::CrosswalkModulePlugin" :
    "autoware::behavior_velocity_planner::CrosswalkModulePlugin"});
  std::unique_ptr<autoware::behavior_velocity_planner::BehaviorVelocityPlannerNode> normal_node;
  std::unique_ptr<autoware::behavior_velocity_planner::experimental::BehaviorVelocityPlannerNode> experimental_node;
  if (experimental) experimental_node = std::make_unique<autoware::behavior_velocity_planner::experimental::BehaviorVelocityPlannerNode>(node);
  else normal_node = std::make_unique<autoware::behavior_velocity_planner::BehaviorVelocityPlannerNode>(node);
  autoware_internal_planning_msgs::PlanningFactorArray::ConstPtr factors;
  autoware_planning_msgs::Path::ConstPtr output;
  auto factor_sub = node.subscribe<autoware_internal_planning_msgs::PlanningFactorArray>(
    "/planning/planning_factors/crosswalk", 1,
    [&](const autoware_internal_planning_msgs::PlanningFactorArray::ConstPtr & msg) { factors=msg; });
  auto path_sub = node.subscribe<autoware_planning_msgs::Path>("/full_crosswalk_host/output/path", 1,
    [&](const autoware_planning_msgs::Path::ConstPtr & msg) { output=msg; });
  const auto root = ros::package::getPath("autoware_behavior_velocity_crosswalk_module") + "/test_config/";
  auto objects = autoware::test_utils::parse<autoware_perception_msgs::PredictedObjects>(
    YAML::LoadFile(root + "dynamic_objects_data.yaml"));
  const auto odometry = autoware::test_utils::parse<nav_msgs::Odometry>(
    YAML::LoadFile(root + "vehicle_odometry_data.yaml"));
  const auto path = autoware::test_utils::parse<std::optional<autoware_internal_planning_msgs::PathWithLaneId>>(
    root + "path_with_lane_id_data.yaml");
  ASSERT_TRUE(path);
  ASSERT_EQ(objects.objects.size(), 1u);
  if (!pedestrian) objects.objects.clear();
  publish(node, "vector_map", autoware::test_utils::makeMapBinMsg());
  publish(node, "accel", geometry_msgs::AccelWithCovarianceStamped{});
  publish(node, "dynamic_objects", objects);
  publish(node, "vehicle_odometry", odometry);
  publish(node, "occupancy_grid", autoware::test_utils::makeCostMapMsg());
  publish(node, "traffic_signals", autoware_perception_msgs::TrafficLightGroupArray{});
  publish(node, "external_velocity_limit_mps", autoware_internal_planning_msgs::VelocityLimit{});
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "map";
  publish(node, "no_ground_pointcloud", cloud_msg);
  publish(node, "path_with_lane_id", *path);
  for (int i=0; i<100 && (!output || !factors); ++i) {
    ros::spinOnce(); ros::WallDuration(0.01).sleep();
  }
  ASSERT_TRUE(output);
  ASSERT_TRUE(factors);
  if (!pedestrian) {
    EXPECT_TRUE(factors->factors.empty());
    return;
  }
  ASSERT_EQ(factors->factors.size(), 1u);
  const auto & factor = factors->factors.front();
  EXPECT_FALSE(factor.safety_factors.is_safe);
  ASSERT_EQ(factor.safety_factors.factors.size(), 1u);
  const auto & safety = factor.safety_factors.factors.front();
  EXPECT_EQ(safety.type, autoware_internal_planning_msgs::SafetyFactor::OBJECT);
  EXPECT_FALSE(safety.is_safe);
  EXPECT_EQ(safety.points.size(), 1u);
  EXPECT_EQ(safety.object_id, objects.objects.front().object_id);
}
TEST(PlanningFactorTest, NodeTestWithPredictedObjects) { runCase(true); }
TEST(PlanningFactorTest, ClearCrosswalkHasNoStopFactor) { runCase(false); }
TEST(PlanningFactorTest, ExperimentalBlockedPedestrian) { runCase(true, true); }
TEST(PlanningFactorTest, ExperimentalClearCrosswalk) { runCase(false, true); }
}
int main(int argc, char ** argv) {
  ros::init(argc, argv, "full_crosswalk_host");
  ros::NodeHandle clock_owner;  // Start ROS before setting the replay clock.
  // Replay the historical fixture on its own clock; wall time would expire
  // RTC states immediately because this recording predates the test run.
  const auto config = ros::package::getPath("autoware_behavior_velocity_crosswalk_module") +
    "/test_config/path_with_lane_id_data.yaml";
  const auto stamp = YAML::LoadFile(config)["header"]["stamp"];
  ros::Time::setNow(ros::Time(stamp["sec"].as<uint32_t>(), stamp["nanosec"].as<uint32_t>()));
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
