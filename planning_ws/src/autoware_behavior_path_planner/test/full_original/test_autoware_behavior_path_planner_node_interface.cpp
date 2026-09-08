// Copyright 2023 TIER IV, Inc.
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

#include "autoware/behavior_path_planner/behavior_path_planner_node.hpp"
#include "autoware/behavior_path_planner/test_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

using autoware::behavior_path_planner::generateNode;
using autoware::behavior_path_planner::generateTestManager;
using autoware::behavior_path_planner::publishMandatoryTopics;

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionRoute)
{

  auto test_manager = generateTestManager();
  auto test_target_node = generateNode({}, {});

  publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_route_topic = "/behavior_path_planner/input/route";

  // test for normal trajectory
  ASSERT_NO_THROW(
    test_manager->behaviorNormalRoute(*test_target_node, input_route_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test with empty route
  ASSERT_NO_THROW(
    test_manager->abnormalRoute(*test_target_node, input_route_topic));

}

TEST(PlanningModuleInterfaceTest, NodeTestWithOffTrackEgoPose)
{


  auto test_manager = generateTestManager();
  auto test_target_node = generateNode({}, {});
  publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_route_topic = "/behavior_path_planner/input/route";
  const std::string input_odometry_topic = "/behavior_path_planner/input/odometry";

  // test for normal trajectory
  ASSERT_NO_THROW(
    test_manager->behaviorNormalRoute(*test_target_node, input_route_topic));

  // make sure behavior_path_planner is running
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  ASSERT_NO_THROW(
    test_manager->offTrackOdometry(*test_target_node, input_odometry_topic));


}

TEST(FullBppNativeHost, MissingRequestedPluginRejectsStartup)
{
  EXPECT_THROW(generateNode({}, {"aw149::DefinitelyMissingSceneManager"}), std::runtime_error);
}

TEST(FullBppNativeHost, EverySceneProviderIsDiscoverableAndConstructible)
{
  pluginlib::ClassLoader<autoware::behavior_path_planner::SceneModuleManagerInterface> loader(
    "autoware_behavior_path_planner_common",
    "autoware::behavior_path_planner::SceneModuleManagerInterface");
  for (const std::string suffix : {
    "SideShiftModuleManager", "SamplingPlannerModuleManager", "StartPlannerModuleManager",
    "LaneChangeLeftModuleManager", "LaneChangeRightModuleManager", "GoalPlannerModuleManager",
    "DynamicObstacleAvoidanceModuleManager", "AvoidanceByLaneChangeModuleManager",
    "BidirectionalTrafficModuleManager", "StaticObstacleAvoidanceModuleManager"}) {
    const std::string name = "autoware::behavior_path_planner::" + suffix;
    ASSERT_TRUE(loader.isClassAvailable(name)) << name;
    ASSERT_TRUE(loader.createUniqueInstance(name)) << name;
  }
}

TEST(FullBppNativeHost, LoadsOriginalSideShiftManagerAndPublishesRoutePath)
{
  auto manager = generateTestManager();
  auto node = generateNode({"side_shift"},
    {"autoware::behavior_path_planner::SideShiftModuleManager"});
  publishMandatoryTopics(manager, node);
  manager->behaviorNormalRoute(*node, "/behavior_path_planner/input/route");
  const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
  while (manager->getReceivedTopicNum() == 0 && ros::WallTime::now() < deadline) {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_GT(manager->getReceivedTopicNum(), 0U);
}

int main(int argc,char ** argv) {
  ros::init(argc,argv,"full_bpp_original_test");
  ::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
