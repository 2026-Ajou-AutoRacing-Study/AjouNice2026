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

#include "autoware/planning_validator/full_source/node.hpp"


#include <autoware/planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>
#include <cmath>

using autoware::planning_test_manager::PlanningInterfaceTestManager;
using autoware::planning_validator::full_source::PlanningValidatorNode;

std::shared_ptr<PlanningInterfaceTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<PlanningInterfaceTestManager>();

  // set subscriber with topic name: planning_validator → test_node_
  test_manager->subscribeOutput<autoware_planning_msgs::Trajectory>(
    "/planning_validator_node/output/trajectory");

  return test_manager;
}

std::shared_ptr<PlanningValidatorNode> generateNode()
{
  ros::NodeHandle node("/planning_validator_node");
  node.setParam("launch_modules",std::vector<std::string>{});
  return std::make_shared<PlanningValidatorNode>(node);
}

void publishMandatoryTopics(
  std::shared_ptr<PlanningInterfaceTestManager> test_manager,
  std::shared_ptr<PlanningValidatorNode> test_target_node)
{
  // publish necessary topics from test_manager
  test_manager->publishInput(
    *test_target_node, "/planning_validator_node/input/kinematics",
    autoware::test_utils::makeOdometry());
  test_manager->publishInput(
    *test_target_node, "/planning_validator_node/input/acceleration",
    geometry_msgs::AccelWithCovarianceStamped{});
  test_manager->publishInput(
    *test_target_node, "/planning_validator_node/input/pointcloud",
    [] { sensor_msgs::PointCloud2 cloud; cloud.header.frame_id="base_link"; return cloud; }());
  test_manager->publishInput(
    *test_target_node, "/planning_validator_node/input/lanelet_map_bin",
    autoware::test_utils::makeMapBinMsg());
  test_manager->publishInput(
    *test_target_node, "/planning_validator_node/input/route",
    autoware::test_utils::makeBehaviorNormalRoute());
}

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectory)
{
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_trajectory_topic = "/planning_validator_node/input/trajectory";

  // test for normal trajectory
  ASSERT_NO_THROW(test_manager->normalTrajectory(*test_target_node, input_trajectory_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for trajectory with empty/one point/overlapping point
  ASSERT_NO_THROW(
    test_manager->abnormalTrajectory(*test_target_node, input_trajectory_topic));
}

TEST(PlanningModuleInterfaceTest, NodeTestWithOffTrackEgoPose)
{
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_trajectory_topic = "/planning_validator_node/input/trajectory";
  const std::string input_odometry_topic = "/planning_validator_node/input/kinematics";

  // test for normal trajectory
  ASSERT_NO_THROW(test_manager->normalTrajectory(*test_target_node, input_trajectory_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for trajectory with empty/one point/overlapping point
  ASSERT_NO_THROW(test_manager->offTrackOdometry(*test_target_node, input_odometry_topic));
}

int main(int argc,char ** argv) {
  ros::init(argc,argv,"full_validator_original_host_tests");
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}

TEST(FullValidatorNativeHost, AllFourOriginalProvidersWithMapRouteAndMotion)
{
  auto manager=generateTestManager();
  ros::NodeHandle options("/planning_validator_node");
  options.setParam("launch_modules",std::vector<std::string>{
    "autoware::planning_validator::full_source::TrajectoryChecker",
    "autoware::planning_validator::full_source::LatencyChecker",
    "autoware::planning_validator::full_source::IntersectionCollisionChecker",
    "autoware::planning_validator::full_source::RearCollisionChecker"});
  auto node=std::make_shared<PlanningValidatorNode>(options);
  publishMandatoryTopics(manager,node);
  autoware_adapi_v1_msgs::OperationModeState mode;
  mode.mode=autoware_adapi_v1_msgs::OperationModeState::AUTONOMOUS;
  mode.is_autoware_control_enabled=true;
  manager->publishInput(*node,"/planning_validator_node/input/operational_mode_state",mode);
  manager->normalTrajectory(*node,"/planning_validator_node/input/trajectory");
  EXPECT_GT(manager->getReceivedTopicNum(),0U);
}

TEST(FullValidatorNativeHost, InvalidIntervalPreviousResultAndSoftStop)
{
  using Trajectory=autoware_planning_msgs::Trajectory;
  using Status=autoware_planning_validator::PlanningValidatorStatus;
  ros::NodeHandle owner;
  for(const int handling : {0,1,2}) {
    SCOPED_TRACE(handling);
    auto manager=generateTestManager();
    ros::NodeHandle options("/planning_validator_node");
    options.setParam("launch_modules",std::vector<std::string>{
      "autoware::planning_validator::full_source::TrajectoryChecker"});
    options.setParam("default_handling_type",handling);
    options.setParam("trajectory_checker/interval/threshold",1.0);
    auto node=std::make_shared<PlanningValidatorNode>(options);
    std::vector<Trajectory> outputs;
    std::vector<Status> statuses;
    auto output_sub=owner.subscribe<Trajectory>("/planning_validator_node/output/trajectory",10,
      [&](const Trajectory::ConstPtr & msg){outputs.push_back(*msg);});
    auto status_sub=owner.subscribe<Status>("/planning_validator_node/output/validation_status",10,
      [&](const Status::ConstPtr & msg){statuses.push_back(*msg);});
    publishMandatoryTopics(manager,node);
    nav_msgs::Odometry odometry;
    odometry.pose.pose.orientation.w=1.0;
    odometry.twist.twist.linear.x=1.0;
    manager->publishInput(*node,"/planning_validator_node/input/kinematics",odometry);
    manager->publishInput(*node,"/localization/kinematic_state",odometry);
    autoware_adapi_v1_msgs::OperationModeState mode;
    mode.mode=autoware_adapi_v1_msgs::OperationModeState::AUTONOMOUS;
    mode.is_autoware_control_enabled=true;
    manager->publishInput(*node,"/planning_validator_node/input/operational_mode_state",mode);
    Trajectory valid;valid.header.frame_id="map";valid.header.stamp=ros::Time::now();
    for(int i=0;i<20;++i) {
      autoware_planning_msgs::TrajectoryPoint point;
      point.pose.position.x=0.5*i;point.pose.orientation.w=1;
      point.longitudinal_velocity_mps=1;valid.points.push_back(point);
    }
    auto invalid=valid;invalid.points.back().pose.position.x+=10;
    const auto send=[&](const Trajectory & trajectory) {
      manager->publishInput(*node,"/planning_validator_node/input/trajectory",trajectory,1);
      for(int i=0;i<30;++i){ros::spinOnce();ros::WallDuration(.01).sleep();}
    };
    // A fallback policy must not fabricate previous output before any valid input.
    send(invalid);
    ASSERT_FALSE(statuses.empty());
    EXPECT_FALSE(statuses.back().is_valid_interval);
    if(handling==0)EXPECT_FALSE(outputs.empty());
    else EXPECT_TRUE(outputs.empty());
    outputs.clear();statuses.clear();
    send(valid);
    ASSERT_FALSE(outputs.empty());ASSERT_FALSE(statuses.empty());
    ASSERT_TRUE(statuses.back().is_valid_interval);
    EXPECT_EQ(statuses.back().invalid_count,0);
    outputs.clear();statuses.clear();
    send(invalid);
    ASSERT_FALSE(outputs.empty());ASSERT_FALSE(statuses.empty());
    EXPECT_FALSE(statuses.back().is_valid_interval);
    EXPECT_GT(statuses.back().invalid_count,0);
    const auto & result=outputs.back();
    ASSERT_EQ(result.points.size(),valid.points.size());
    EXPECT_DOUBLE_EQ(result.points.back().pose.position.x,
      handling==0 ? invalid.points.back().pose.position.x : valid.points.back().pose.position.x);
    if(handling==1)EXPECT_FLOAT_EQ(result.points.back().longitudinal_velocity_mps,1.0F);
    if(handling==2) {
      EXPECT_LT(result.points.back().longitudinal_velocity_mps,1.0F);
      for(const auto & point:result.points) {
        EXPECT_TRUE(std::isfinite(point.longitudinal_velocity_mps));
        EXPECT_GE(point.longitudinal_velocity_mps,0.0F);
        EXPECT_LE(point.longitudinal_velocity_mps,1.0F);
      }
    }
  }
}
