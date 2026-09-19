// Copyright 2025 TIER IV, Inc.
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

#include "obstacle_stop_module.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

class PlanningFactorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {

    node_=std::make_shared<ros::NodeHandle>("/full_motion_obstacle_test");
    node_->setParam("obstacle_stop/obstacle_filtering/check_inside/pointcloud",true);
    module_ = std::make_unique<autoware::motion_velocity_planner::ObstacleStopModule>();

    module_->init(*node_, "obstacle_stop");
  }

  void TearDown() override {}

  std::unique_ptr<autoware::motion_velocity_planner::ObstacleStopModule> module_;
  std::shared_ptr<ros::NodeHandle> node_;
  nav_msgs::Odometry odometry_;
  autoware_planning_msgs::Trajectory trajectory_;

  void InitializeEgoData()
  {
    // Set odometry
    odometry_.header.frame_id = "map";
    odometry_.header.stamp = ros::Time::now();
    odometry_.pose.pose.position.x = 0.0;
    odometry_.pose.pose.position.y = 0.0;
    odometry_.pose.pose.position.z = 0.0;
    odometry_.pose.pose.orientation.w = 1.0;

    // Generate trajectory points along a straight line
    trajectory_.header.frame_id = "map";
    trajectory_.header.stamp = ros::Time::now();

    for (double x = 0.0; x <= 100.0; x += 1.0) {
      autoware_planning_msgs::TrajectoryPoint point;
      point.pose.position.x = x;
      point.pose.position.y = 0.0;
      point.pose.position.z = 0.0;
      point.pose.orientation.w = 1.0;
      point.longitudinal_velocity_mps = 10.0;
      trajectory_.points.push_back(point);
    }
  }
};

TEST_F(PlanningFactorTest, TestWithPredictedObjects)
{
  InitializeEgoData();

  // Add a simple pedestrian obstacle for testing
  autoware_perception_msgs::PredictedObject pedestrian;
  pedestrian.existence_probability = 1.0;

  // Set classification
  autoware_perception_msgs::ObjectClassification classification;
  classification.label = autoware_perception_msgs::ObjectClassification::PEDESTRIAN;
  classification.probability = 1.0;
  pedestrian.classification.resize(1);
  pedestrian.classification.at(0) = classification;

  // Set pose - place pedestrian in front of ego
  geometry_msgs::Pose initial_pose;
  initial_pose.position.x = 20.0;
  initial_pose.position.y = 0.0;
  initial_pose.position.z = 0.0;
  initial_pose.orientation.w = 1.0;
  pedestrian.kinematics.initial_pose_with_covariance.pose = initial_pose;

  geometry_msgs::Twist initial_twist;
  initial_twist.linear.x = 0.0;
  initial_twist.linear.y = 0.0;
  pedestrian.kinematics.initial_twist_with_covariance.twist = initial_twist;

  // Set shape
  pedestrian.shape.type = autoware_perception_msgs::Shape::CYLINDER;
  pedestrian.shape.dimensions.x = 0.5;
  pedestrian.shape.dimensions.y = 0.5;
  pedestrian.shape.dimensions.z = 1.8;

  pedestrian.kinematics.predicted_paths.resize(1);
  auto & predicted_path = pedestrian.kinematics.predicted_paths.front();
  predicted_path.path.resize(10);
  predicted_path.time_step.sec=ros::Duration(0.5).sec; predicted_path.time_step.nanosec=ros::Duration(0.5).nsec;
  predicted_path.confidence = 1.0;
  for (size_t i = 0; i < predicted_path.path.size(); ++i) {
    predicted_path.path[i].position.x = initial_pose.position.x + i * 0.5 * initial_twist.linear.x;
    predicted_path.path[i].position.y = initial_pose.position.y + i * 0.5 * initial_twist.linear.y;
    predicted_path.path[i].position.z = initial_pose.position.z;
    predicted_path.path[i].orientation = initial_pose.orientation;
  }

  autoware_perception_msgs::PredictedObjects predicted_objects;
  predicted_objects.header.frame_id = "map";
  predicted_objects.header.stamp = ros::Time::now();
  predicted_objects.objects.push_back(pedestrian);

  auto planner_data = std::make_shared<autoware::motion_velocity_planner::PlannerData>(*node_);
  planner_data->current_odometry = odometry_;
  planner_data->process_predicted_objects(predicted_objects);

  module_->plan(trajectory_.points, trajectory_.points, planner_data);

  const auto planning_factors = module_->get_planning_factors();
  ASSERT_EQ(planning_factors.size(), 1);
  const auto & planning_factor = planning_factors.front();
  EXPECT_EQ(planning_factor.behavior, autoware_internal_planning_msgs::PlanningFactor::STOP);
  EXPECT_NEAR(planning_factor.control_points.front().pose.position.x, 10.0, 3.0);
  EXPECT_NEAR(planning_factor.control_points.front().pose.position.y, 0.0, 1.0);
  EXPECT_NEAR(planning_factor.control_points.front().pose.position.z, 0.0, 1.0);

  ASSERT_EQ(planning_factor.safety_factors.factors.size(), 1);
  const auto & safety_factor = planning_factor.safety_factors.factors.front();
  EXPECT_EQ(safety_factor.type, autoware_internal_planning_msgs::SafetyFactor::OBJECT);
  EXPECT_FALSE(safety_factor.is_safe);
  ASSERT_EQ(safety_factor.points.size(), 1);
  EXPECT_NEAR(safety_factor.points.front().x, initial_pose.position.x, 1.0);
  EXPECT_NEAR(safety_factor.points.front().y, initial_pose.position.y, 1.0);
  EXPECT_NEAR(safety_factor.points.front().z, initial_pose.position.z, 1.0);
  EXPECT_EQ(safety_factor.object_id, predicted_objects.objects.front().object_id);
}

TEST_F(PlanningFactorTest, TestWithPointCloudObstacle)
{
  InitializeEgoData();

  auto planner_data = std::make_shared<autoware::motion_velocity_planner::PlannerData>(*node_);
  planner_data->current_odometry = odometry_;

  const auto count_param_str =
    "obstacle_stop.pointcloud_segmentation.velocity_estimation.required_velocity_count";
  const size_t count = autoware::motion_velocity_planner::motion_obstacle_stop_ros1::require_parameter<int>(*node_,count_param_str);
  const auto time_us = static_cast<uint64_t>(
    ros::Time::now().toSec() * 1e6 + ros::Time::now().toNSec() / 1e3);
  constexpr uint64_t pointcloud_period_us = 1e5;

  const float point_x = 20.0f;
  const float point_y = 0.0f;
  const float point_z = 1.0f;

  for (size_t i = 0; i < count + 1; ++i) {
    module_->publishPlanningFactor();

    // prepare point cloud obstacle
    pcl::PointCloud<pcl::PointXYZ>::Ptr single_point(new pcl::PointCloud<pcl::PointXYZ>);
    single_point->header.stamp = time_us + i * pointcloud_period_us;
    single_point->width = 1;
    single_point->height = 1;
    single_point->is_dense = true;
    single_point->header.frame_id = "map";
    single_point->points.resize(1);
    single_point->points[0].x = point_x;
    single_point->points[0].y = point_y;
    single_point->points[0].z = point_z;

    planner_data->no_ground_pointcloud.preprocess_pointcloud(
      std::move(*single_point), trajectory_.points, odometry_, 100.0, planner_data->vehicle_info_,
      planner_data->trajectory_polygon_collision_check, planner_data->ego_nearest_dist_threshold,
      planner_data->ego_nearest_yaw_threshold);

    module_->plan(trajectory_.points, trajectory_.points, planner_data);
  }

  const auto planning_factors = module_->get_planning_factors();
  ASSERT_EQ(planning_factors.size(), 1);
  const auto & planning_factor = planning_factors.front();
  EXPECT_EQ(planning_factor.behavior, autoware_internal_planning_msgs::PlanningFactor::STOP);
  EXPECT_NEAR(planning_factor.control_points.front().pose.position.x, 10.0, 3.0);
  EXPECT_NEAR(planning_factor.control_points.front().pose.position.y, 0.0, 1.0);
  EXPECT_NEAR(planning_factor.control_points.front().pose.position.z, 0.0, 1.0);

  ASSERT_EQ(planning_factor.safety_factors.factors.size(), 1);
  const auto & safety_factor = planning_factor.safety_factors.factors.front();
  EXPECT_EQ(safety_factor.type, autoware_internal_planning_msgs::SafetyFactor::POINTCLOUD);
  EXPECT_FALSE(safety_factor.is_safe);
  ASSERT_EQ(safety_factor.points.size(), 1);
  EXPECT_NEAR(safety_factor.points.front().x, point_x, 1.0);
  EXPECT_NEAR(safety_factor.points.front().y, point_y, 1.0);
  EXPECT_NEAR(safety_factor.points.front().z, point_z, 1.0);
}
