// Copyright 2023 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/behavior_velocity_planner/test_utils.hpp"
#include "autoware/behavior_velocity_planner/experimental/full_node.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionPathWithLaneID)
{

  auto test_manager = autoware::behavior_velocity_planner::generateTestManager();
  auto test_target_node = autoware::behavior_velocity_planner::generateNode({});

  autoware::behavior_velocity_planner::publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_path_with_lane_id_topic =
    "/behavior_velocity_planner_node/input/path_with_lane_id";

  // test with nominal path_with_lane_id
  ASSERT_NO_THROW(
    test_manager->normalPathWithLaneId(*test_target_node, input_path_with_lane_id_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test with empty path_with_lane_id
  ASSERT_NO_THROW(
    test_manager->abnormalPathWithLaneId(*test_target_node, input_path_with_lane_id_topic));

}

TEST(PlanningModuleInterfaceTest, NodeTestWithOffTrackEgoPose)
{


  auto test_manager = autoware::behavior_velocity_planner::generateTestManager();
  auto test_target_node = autoware::behavior_velocity_planner::generateNode({});
  autoware::behavior_velocity_planner::publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_path_with_lane_id_topic =
    "/behavior_velocity_planner_node/input/path_with_lane_id";
  const std::string input_odometry_topic = "/behavior_velocity_planner_node/input/vehicle_odometry";

  // test for normal trajectory
  ASSERT_NO_THROW(
    test_manager->normalPathWithLaneId(*test_target_node, input_path_with_lane_id_topic));

  // make sure behavior_path_planner is running
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  ASSERT_NO_THROW(
    test_manager->offTrackOdometry(*test_target_node, input_odometry_topic));


}
TEST(FullBvpNativeContract, RejectsExplicitMissingPlugin)
{
  auto configured = generateNode({});
  configured.reset();
  ros::NodeHandle private_node("/behavior_velocity_planner_node");
  private_node.setParam("launch_modules", std::vector<std::string>{"aw149::MissingBvpPlugin"});
  try {
    FullBehaviorVelocityPlannerNode node(private_node);
    FAIL() << "Missing requested plugin must reject startup";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("Required BVP plugin unavailable"), std::string::npos);
  }
}

TEST(FullBvpNativeContract, NoOutputWithoutMandatoryInputs)
{
  auto manager = generateTestManager();
  auto node = generateNode({});
  manager->normalPathWithLaneId(*node, "/behavior_velocity_planner_node/input/path_with_lane_id");
  EXPECT_EQ(manager->getReceivedTopicNum(), 0U);
}

TEST(FullBvpNativeContract, RealDetectionAreaPluginRunsInFullHost)
{
  auto manager = generateTestManager();
  auto node = generateNode({{"detection_area",
    "autoware::behavior_velocity_planner::DetectionAreaModulePlugin"}});
  publishMandatoryTopics(manager, node);
  manager->normalPathWithLaneId(*node, "/behavior_velocity_planner_node/input/path_with_lane_id");
  EXPECT_GT(manager->getReceivedTopicNum(), 0U);
}

TEST(FullBvpNativeContract, NoOutputWithoutMapEvenWithMotionState)
{
  auto manager = generateTestManager();
  auto node = generateNode({});
  manager->publishInput(*node, "/behavior_velocity_planner_node/input/vehicle_odometry",
    autoware::test_utils::makeOdometry());
  manager->publishInput(*node, "/behavior_velocity_planner_node/input/accel",
    geometry_msgs::AccelWithCovarianceStamped{});
  manager->normalPathWithLaneId(*node, "/behavior_velocity_planner_node/input/path_with_lane_id");
  EXPECT_EQ(manager->getReceivedTopicNum(), 0U);
}

TEST(FullBvpNativeContract, NoStoppingAreaWaitsForObjectMessageThenProducesPath)
{
  auto manager = generateTestManager();
  auto node = generateNode({{"no_stopping_area",
    "autoware::behavior_velocity_planner::NoStoppingAreaModulePlugin"}});
  manager->publishInput(*node, "/behavior_velocity_planner_node/input/vehicle_odometry",
    autoware::test_utils::makeOdometry());
  manager->publishInput(*node, "/behavior_velocity_planner_node/input/accel",
    geometry_msgs::AccelWithCovarianceStamped{});
  manager->publishInput(*node, "/behavior_velocity_planner_node/input/vector_map",
    autoware::test_utils::makeMapBinMsg());
  manager->normalPathWithLaneId(*node, "/behavior_velocity_planner_node/input/path_with_lane_id");
  EXPECT_EQ(manager->getReceivedTopicNum(), 0U);
  manager->publishInput(*node, "/behavior_velocity_planner_node/input/dynamic_objects",
    autoware_perception_msgs::PredictedObjects{});
  manager->normalPathWithLaneId(*node, "/behavior_velocity_planner_node/input/path_with_lane_id");
  EXPECT_GT(manager->getReceivedTopicNum(), 0U);
}

TEST(FullBvpNativeContract, ExperimentalRequiresMapMotionStateAndProducesPath)
{
  // Load identical production fixture parameters, then own only the experimental host.
  auto normal = generateNode({});
  normal.reset();
  ros::NodeHandle private_node("/behavior_velocity_planner_node");
  experimental::FullBehaviorVelocityPlannerNode node(private_node);
  auto manager = generateTestManager();
  manager->normalPathWithLaneId(node, "/behavior_velocity_planner_node/input/path_with_lane_id");
  EXPECT_EQ(manager->getReceivedTopicNum(), 0U);
  manager->publishInput(node, "/behavior_velocity_planner_node/input/vehicle_odometry",
    autoware::test_utils::makeOdometry());
  manager->publishInput(node, "/behavior_velocity_planner_node/input/accel",
    geometry_msgs::AccelWithCovarianceStamped{});
  manager->publishInput(node, "/behavior_velocity_planner_node/input/vector_map",
    autoware::test_utils::makeMapBinMsg());
  manager->normalPathWithLaneId(node, "/behavior_velocity_planner_node/input/path_with_lane_id");
  EXPECT_GT(manager->getReceivedTopicNum(), 0U);
}

TEST(FullBvpNativeContract, ExperimentalDetectionWaitsForObjectsThenProducesPath)
{
  auto configured = generateNode({{"detection_area",
    "autoware::behavior_velocity_planner::DetectionAreaModulePlugin"}});
  configured.reset();
  ros::NodeHandle private_node("/behavior_velocity_planner_node");
  private_node.setParam("launch_modules", std::vector<std::string>{
    "autoware::behavior_velocity_planner::experimental::DetectionAreaModulePlugin"});
  experimental::FullBehaviorVelocityPlannerNode node(private_node);
  auto manager = generateTestManager();
  manager->publishInput(node, "/behavior_velocity_planner_node/input/vehicle_odometry",
    autoware::test_utils::makeOdometry());
  manager->publishInput(node, "/behavior_velocity_planner_node/input/accel",
    geometry_msgs::AccelWithCovarianceStamped{});
  manager->publishInput(node, "/behavior_velocity_planner_node/input/vector_map",
    autoware::test_utils::makeMapBinMsg());
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  manager->publishInput(node, "/behavior_velocity_planner_node/input/no_ground_pointcloud", cloud);
  manager->normalPathWithLaneId(node, "/behavior_velocity_planner_node/input/path_with_lane_id");
  EXPECT_EQ(manager->getReceivedTopicNum(), 0U);
  manager->publishInput(node, "/behavior_velocity_planner_node/input/dynamic_objects",
    autoware_perception_msgs::PredictedObjects{});
  manager->normalPathWithLaneId(node, "/behavior_velocity_planner_node/input/path_with_lane_id");
  EXPECT_GT(manager->getReceivedTopicNum(), 0U);
}

}  // namespace autoware::behavior_velocity_planner

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "full_bvp_original_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
