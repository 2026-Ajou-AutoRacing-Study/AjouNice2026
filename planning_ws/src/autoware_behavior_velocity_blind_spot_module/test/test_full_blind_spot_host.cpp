// Real BVP hosts with original Blind Spot recorded map/path/object inputs.
// New bounded assertions: these are not the upstream-disabled legacy tests.
#include <autoware/behavior_velocity_planner/node.hpp>
#include <autoware/behavior_velocity_planner/experimental/node.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>
#include <autoware_planning_msgs/Path.h>
#include <std_msgs/String.h>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
namespace {
template<class T> void publish(ros::NodeHandle & node, const std::string & topic, const T & value)
{
  auto publisher = node.advertise<T>("/full_blind_spot_host/input/" + topic, 1, true);
  for (int i=0; i<100 && !publisher.getNumSubscribers(); ++i) {
    ros::spinOnce(); ros::WallDuration(0.01).sleep();
  }
  ASSERT_GT(publisher.getNumSubscribers(), 0u) << topic;
  for (int i=0; i<5; ++i) {
    publisher.publish(value); ros::WallDuration(0.04).sleep(); ros::spinOnce();
  }
}
void runCase(const std::string & fixture, bool experimental)
{
  ros::NodeHandle node;
  const auto config = YAML::LoadFile(ros::package::getPath("autoware_behavior_velocity_blind_spot_module") + "/test_data/" + fixture);
  const auto path = autoware::test_utils::parse<autoware_internal_planning_msgs::PathWithLaneId>(config["path_with_lane_id"]);
  ASSERT_GT(path.points.size(), 2u);
  ros::Time::setNow(path.header.stamp);
  node.setParam("/full_blind_spot_host/launch_modules", std::vector<std::string>{experimental ?
    "autoware::behavior_velocity_planner::experimental::BlindSpotModulePlugin" :
    "autoware::behavior_velocity_planner::BlindSpotModulePlugin"});
  std::unique_ptr<autoware::behavior_velocity_planner::BehaviorVelocityPlannerNode> normal;
  std::unique_ptr<autoware::behavior_velocity_planner::experimental::BehaviorVelocityPlannerNode> exp;
  if (experimental) exp = std::make_unique<autoware::behavior_velocity_planner::experimental::BehaviorVelocityPlannerNode>(node);
  else normal = std::make_unique<autoware::behavior_velocity_planner::BehaviorVelocityPlannerNode>(node);
  autoware_planning_msgs::Path::ConstPtr output;
  std_msgs::String::ConstPtr decision;
  auto path_sub = node.subscribe<autoware_planning_msgs::Path>("/full_blind_spot_host/output/path", 1,
    [&](const autoware_planning_msgs::Path::ConstPtr & msg){output=msg;});
  auto decision_sub = node.subscribe<std_msgs::String>("/full_blind_spot_host/debug/blind_spot/decision_state", 1,
    [&](const std_msgs::String::ConstPtr & msg){decision=msg;});
  const auto map_path = autoware::test_utils::resolve_pkg_share_uri(config["map_path_uri"].as<std::string>());
  ASSERT_TRUE(map_path);
  publish(node, "vector_map", autoware::test_utils::make_map_bin_msg(*map_path));
  publish(node, "accel", geometry_msgs::AccelWithCovarianceStamped{});
  publish(node, "dynamic_objects", autoware::test_utils::parse<autoware_perception_msgs::PredictedObjects>(config["dynamic_object"]));
  publish(node, "vehicle_odometry", autoware::test_utils::parse<nav_msgs::Odometry>(config["self_odometry"]));
  publish(node, "occupancy_grid", autoware::test_utils::makeCostMapMsg());
  publish(node, "traffic_signals", autoware_perception_msgs::TrafficLightGroupArray{});
  publish(node, "external_velocity_limit_mps", autoware_internal_planning_msgs::VelocityLimit{});
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 cloud_msg; pcl::toROSMsg(cloud, cloud_msg); cloud_msg.header.frame_id = "map";
  publish(node, "no_ground_pointcloud", cloud_msg);
  publish(node, "path_with_lane_id", path);
  ASSERT_TRUE(output);
  ASSERT_FALSE(output->points.empty());
  ASSERT_TRUE(decision) << "No actual Blind Spot scene decision was published";
  EXPECT_EQ(decision->data.find("InternalError"), std::string::npos) << decision->data;
  double max_input = 0;
  for (const auto & point : path.points) max_input = std::max(max_input, double(point.point.longitudinal_velocity_mps));
  for (const auto & point : output->points) {
    EXPECT_TRUE(std::isfinite(point.longitudinal_velocity_mps));
    EXPECT_GE(point.longitudinal_velocity_mps, 0);
    EXPECT_LE(point.longitudinal_velocity_mps, max_input + 1e-4);
  }
  ROS_INFO_STREAM("Original fixture " << fixture << " decision: " << decision->data);
}
TEST(FullBlindSpotHost, AdjacentNormal) { runCase("object_on_adj_lane.yaml", false); }
TEST(FullBlindSpotHost, ShoulderNormal) { runCase("object_on_shoulder.yaml", false); }
TEST(FullBlindSpotHost, AdjacentExperimental) { runCase("object_on_adj_lane.yaml", true); }
TEST(FullBlindSpotHost, ShoulderExperimental) { runCase("object_on_shoulder.yaml", true); }
}
int main(int argc, char ** argv) {
  ros::init(argc, argv, "full_blind_spot_host");
  ros::NodeHandle clock_owner;
  testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS();
}
