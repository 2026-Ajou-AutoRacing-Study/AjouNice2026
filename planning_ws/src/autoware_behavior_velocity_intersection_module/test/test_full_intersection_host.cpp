// Native ROS1 transport for upstream test_planning_factor.cpp's two fixtures.
// Preserve original maps/data and every original control/safety expectation.
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
#include <algorithm>

namespace {
template<class T> void publish(ros::NodeHandle & node, const std::string & name, const T & value)
{
  auto publisher = node.advertise<T>("/full_intersection_host/input/" + name, 1, true);
  for (int i=0; i<100 && !publisher.getNumSubscribers(); ++i) {
    ros::spinOnce(); ros::WallDuration(0.01).sleep();
  }
  ASSERT_GT(publisher.getNumSubscribers(), 0u) << name;
  for (int i=0; i<5; ++i) {
    publisher.publish(value); ros::WallDuration(0.04).sleep(); ros::spinOnce();
  }
}
nav_msgs::OccupancyGrid loadGrid(const std::string & filename)
{
  const auto node = YAML::LoadFile(filename);
  nav_msgs::OccupancyGrid grid;
  grid.header = autoware::test_utils::parse<std_msgs::Header>(node["header"]);
  const auto info = node["info"];
  grid.info.map_load_time = ros::Time(info["map_load_time"]["sec"].as<uint32_t>(),
                                    info["map_load_time"]["nanosec"].as<uint32_t>());
  grid.info.resolution = info["resolution"].as<float>();
  grid.info.width = info["width"].as<uint32_t>();
  grid.info.height = info["height"].as<uint32_t>();
  grid.info.origin = autoware::test_utils::parse<geometry_msgs::Pose>(info["origin"]);
  // yaml-cpp on Noetic treats int8_t as a character, not a numeric scalar.
  // Parse integers and range-check before preserving their exact byte values.
  for (const auto & value : node["data"]) {
    const int cell = value.as<int>();
    if (cell < -1 || cell > 100) throw std::runtime_error("invalid occupancy cell in source fixture");
    grid.data.push_back(static_cast<int8_t>(cell));
  }
  return grid;
}
void runCase(bool occlusion, bool experimental)
{
  ros::NodeHandle node;
  const auto root = ros::package::getPath("autoware_behavior_velocity_intersection_module") + "/test_data/";
  const std::string suffix = occlusion ? "_for_occlusion_data.yaml" : "_data.yaml";
  const auto objects = autoware::test_utils::parse<autoware_perception_msgs::PredictedObjects>(
    YAML::LoadFile(root + "dynamic_objects" + suffix));
  const auto odometry = autoware::test_utils::parse<nav_msgs::Odometry>(
    YAML::LoadFile(root + "vehicle_odometry" + suffix));
  const auto path = autoware::test_utils::parse<std::optional<autoware_internal_planning_msgs::PathWithLaneId>>(
    root + "path_with_lane_id" + suffix);
  ASSERT_TRUE(path);
  // Replay historical messages at their original clock, not today's wall time.
  ros::Time::setNow(path->header.stamp);
  node.setParam("/full_intersection_host/launch_modules", std::vector<std::string>{experimental ?
    "autoware::behavior_velocity_planner::experimental::IntersectionModulePlugin" :
    "autoware::behavior_velocity_planner::IntersectionModulePlugin"});
  std::unique_ptr<autoware::behavior_velocity_planner::BehaviorVelocityPlannerNode> normal_node;
  std::unique_ptr<autoware::behavior_velocity_planner::experimental::BehaviorVelocityPlannerNode> experimental_node;
  if (experimental) experimental_node = std::make_unique<autoware::behavior_velocity_planner::experimental::BehaviorVelocityPlannerNode>(node);
  else normal_node = std::make_unique<autoware::behavior_velocity_planner::BehaviorVelocityPlannerNode>(node);
  autoware_internal_planning_msgs::PlanningFactorArray::ConstPtr factors;
  autoware_planning_msgs::Path::ConstPtr output;
  auto factor_sub = node.subscribe<autoware_internal_planning_msgs::PlanningFactorArray>(
    std::string("/planning/planning_factors/") + (occlusion ? "intersection_occlusion" : "intersection"), 1,
    [&](const autoware_internal_planning_msgs::PlanningFactorArray::ConstPtr & msg) { factors=msg; });
  auto path_sub = node.subscribe<autoware_planning_msgs::Path>("/full_intersection_host/output/path", 1,
    [&](const autoware_planning_msgs::Path::ConstPtr & msg) { output=msg; });
  publish(node, "vector_map", autoware::test_utils::makeMapBinMsg("autoware_test_utils",
    occlusion ? "intersection/lanelet2_map.osm" : "lanelet2_map.osm"));
  publish(node, "accel", geometry_msgs::AccelWithCovarianceStamped{});
  publish(node, "dynamic_objects", objects);
  publish(node, "vehicle_odometry", odometry);
  publish(node, "occupancy_grid", occlusion ? loadGrid(root + "occupancy_grid_for_occlusion_data.yaml") :
          autoware::test_utils::makeCostMapMsg());
  publish(node, "traffic_signals", autoware_perception_msgs::TrafficLightGroupArray{});
  publish(node, "external_velocity_limit_mps", autoware_internal_planning_msgs::VelocityLimit{});
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg); cloud_msg.header.frame_id = "map";
  publish(node, "no_ground_pointcloud", cloud_msg);
  publish(node, "path_with_lane_id", *path);
  for (int i=0; i<100 && (!output || !factors); ++i) {
    ros::spinOnce(); ros::WallDuration(0.01).sleep();
  }
  ASSERT_TRUE(output);
  ASSERT_TRUE(factors);
  ASSERT_EQ(factors->factors.size(), 1u);
  const auto & factor = factors->factors.front();
  EXPECT_FALSE(factor.safety_factors.is_safe);
  ASSERT_EQ(factor.control_points.size(), 1u);
  EXPECT_LT(factor.control_points.front().distance, 30.0);
  if (occlusion) {
    EXPECT_EQ(factor.safety_factors.factors.size(), 0u);
  } else {
    ASSERT_EQ(factor.safety_factors.factors.size(), 1u);
    const auto & safety = factor.safety_factors.factors.front();
    EXPECT_EQ(safety.type, autoware_internal_planning_msgs::SafetyFactor::OBJECT);
    EXPECT_FALSE(safety.is_safe);
    EXPECT_EQ(safety.points.size(), 1u);
    EXPECT_TRUE(std::any_of(objects.objects.begin(), objects.objects.end(),
      [&](const auto & object) { return object.object_id == safety.object_id; }));
  }
}
TEST(PlanningFactorTest, NodeTestWithPredictedObjects) { runCase(false, false); }
TEST(PlanningFactorTest, NodeTestWithPredictedObjectsForOcclusion) { runCase(true, false); }
TEST(PlanningFactorTest, ExperimentalCollision) { runCase(false, true); }
TEST(PlanningFactorTest, ExperimentalOcclusion) { runCase(true, true); }
}
int main(int argc, char ** argv) {
  ros::init(argc, argv, "full_intersection_host");
  ros::NodeHandle clock_owner;
  testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS();
}
