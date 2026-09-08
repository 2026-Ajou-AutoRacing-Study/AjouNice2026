// Copyright 2024 TIER IV, Inc.
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

#include "autoware/costmap_generator/costmap_generator.hpp"

#include <ros/package.h>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <ros/ros.h>

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <vector>

using autoware::costmap_generator::CostmapGenerator;
using autoware_internal_planning_msgs::Scenario;

class TestCostmapGenerator : public ::testing::Test
{
public:
  void SetUp() override
  {
    
    set_up_node();
    setup_lanelet_map();
  }

  void set_up_node()
  {
    ros::NodeHandle node("/costmap_original_tests");
    costmap_generator_ = std::make_shared<CostmapGenerator>(node);
  }

  void setup_lanelet_map()
  {
    const auto lanelet2_path = autoware::test_utils::get_absolute_path_to_lanelet_map(
      "autoware_test_utils", "overlap_map.osm");
    const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(lanelet2_path, 5.0);
    lanelet_map_ = lanelet_map_ = autoware::experimental::lanelet2_utils::remove_const(
      autoware::experimental::lanelet2_utils::from_autoware_map_msgs(map_bin_msg));
  }

  [[nodiscard]] double get_grid_length_x() { return costmap_generator_->costmap_.getLength()[0]; }

  [[nodiscard]] double get_grid_length_y() { return costmap_generator_->costmap_.getLength()[1]; }

  [[nodiscard]] double get_grid_resolution()
  {
    return costmap_generator_->costmap_.getResolution();
  }

  [[nodiscard]] std::vector<std::string> get_grid_layers()
  {
    return costmap_generator_->costmap_.getLayers();
  }

  size_t test_load_road_areas()
  {
    std::vector<geometry_msgs::Polygon> road_areas;
    costmap_generator_->loadRoadAreasFromLaneletMap(lanelet_map_, road_areas);
    return road_areas.size();
  }

  size_t test_load_parking_areas()
  {
    std::vector<geometry_msgs::Polygon> parking_areas;
    costmap_generator_->loadParkingAreasFromLaneletMap(lanelet_map_, parking_areas);
    return parking_areas.size();
  }

  bool test_is_active_by_pos(
    const bool is_within_parking_lot = true, const bool no_lanelet_map = false)
  {
    costmap_generator_->lanelet_map_.reset();
    if (no_lanelet_map) {
      return costmap_generator_->isActive();
    }

    costmap_generator_->lanelet_map_ = lanelet_map_;
    costmap_generator_->param_->activate_by_scenario = false;

    geometry_msgs::PoseStamped::Ptr p(new geometry_msgs::PoseStamped());
    p->pose.position.x = 3697.07;
    p->pose.position.y = 73735.49;
    if (!is_within_parking_lot) {
      p->pose.position.x += 10.0;
    }

    costmap_generator_->current_pose_ = p;
    return costmap_generator_->isActive();
  }

  bool test_is_active_by_scenario(
    const bool is_parking_scenario = true, const bool no_lanelet_map = false)
  {
    costmap_generator_->lanelet_map_.reset();
    if (no_lanelet_map) {
      return costmap_generator_->isActive();
    }

    costmap_generator_->lanelet_map_ = lanelet_map_;
    costmap_generator_->param_->activate_by_scenario = true;

    Scenario scenario;
    scenario.current_scenario = is_parking_scenario ? Scenario::PARKING : Scenario::LANEDRIVING;
    if (is_parking_scenario) {
      scenario.activating_scenarios.push_back(Scenario::PARKING);
    }

    costmap_generator_->scenario_ = boost::make_shared<Scenario>(scenario);
    return costmap_generator_->isActive();
  }

  void test_publish_combined_costmap()
  {
    // Exercise the restored timer with actual map, point/object layers and
    // TF conversion; inspect the real ROS OccupancyGrid publication.
    const auto path = autoware::test_utils::get_absolute_path_to_lanelet_map(
      "autoware_test_utils", "overlap_map.osm");
    const auto map = autoware::test_utils::make_map_bin_msg(path, 5.0);
    costmap_generator_->onLaneletMapBin(boost::make_shared<autoware_map_msgs::LaneletMapBin>(map));
    costmap_generator_->param_->activate_by_scenario = true;
    auto scenario = boost::make_shared<Scenario>();
    scenario->activating_scenarios = {Scenario::PARKING};
    costmap_generator_->pending_scenario_ = scenario;
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.header.stamp = ros::Time::now();
    transform.transform.translation.x = 3697.07;
    transform.transform.translation.y = 73735.49;
    transform.transform.rotation.w = 1;
    ASSERT_TRUE(costmap_generator_->tf_buffer_.setTransform(transform, "costmap_test", true));
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.emplace_back(3697.07F, 73735.49F, 1.0F);
    auto points = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(cloud, *points);
    points->header.frame_id = "map";
    costmap_generator_->pending_points_ = points;
    auto objects = boost::make_shared<autoware_perception_msgs::PredictedObjects>();
    objects->header.frame_id = "map";
    autoware_perception_msgs::PredictedObject object;
    object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
    object.shape.dimensions.x = 4;
    object.shape.dimensions.y = 2;
    object.kinematics.initial_pose_with_covariance.pose.position.x = 3707.07;
    object.kinematics.initial_pose_with_covariance.pose.position.y = 73735.49;
    object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1;
    autoware_perception_msgs::ObjectClassification classification;
    classification.label = autoware_perception_msgs::ObjectClassification::CAR;
    classification.probability = 0.8;
    object.classification = {classification};
    objects->objects = {object};
    costmap_generator_->pending_objects_ = objects;
    nav_msgs::OccupancyGridConstPtr received;
    ros::NodeHandle node("/costmap_original_tests");
    const auto subscriber = node.subscribe<nav_msgs::OccupancyGrid>("output/occupancy_grid", 1,
      [&](const nav_msgs::OccupancyGridConstPtr & message) { received = message; });
    const auto deadline = ros::WallTime::now() + ros::WallDuration(3);
    while (!received && ros::WallTime::now() < deadline) {
      costmap_generator_->onTimer();
      ros::spinOnce();
      ros::WallDuration(0.02).sleep();
    }
    ASSERT_TRUE(received);
    EXPECT_EQ(received->header.frame_id, "map");
    EXPECT_EQ(received->data.size(), received->info.width * received->info.height);
    EXPECT_NE(std::find(received->data.begin(), received->data.end(), 100), received->data.end());
    const auto & grid = costmap_generator_->costmap_;
    EXPECT_FLOAT_EQ(grid["points"].maxCoeff(), 1);
    EXPECT_FLOAT_EQ(grid["objects"].maxCoeff(), 0.8F);
    EXPECT_TRUE((grid["combined"].array() >= grid["points"].array()).all());
    EXPECT_TRUE((grid["combined"].array() >= grid["objects"].array()).all());
    EXPECT_TRUE((grid["combined"].array() >= grid["primitives"].array()).all());
  }

  void TearDown() override
  {
    
    costmap_generator_ = nullptr;
    lanelet_map_ = nullptr;
  }

  std::shared_ptr<CostmapGenerator> costmap_generator_;
  lanelet::LaneletMapPtr lanelet_map_;
};

TEST_F(TestCostmapGenerator, testInitializeGridmap)
{
  const double grid_resolution = get_grid_resolution();
  const double grid_length_x = get_grid_length_x();
  const double grid_length_y = get_grid_length_y();
  EXPECT_FLOAT_EQ(grid_resolution, 0.3);
  EXPECT_TRUE(70.0 - grid_length_x < grid_resolution);
  EXPECT_TRUE(70.0 - grid_length_y < grid_resolution);

  const auto grid_layers = get_grid_layers();

  EXPECT_TRUE(std::find(grid_layers.begin(), grid_layers.end(), "points") != grid_layers.end());
  EXPECT_TRUE(std::find(grid_layers.begin(), grid_layers.end(), "objects") != grid_layers.end());
  EXPECT_TRUE(std::find(grid_layers.begin(), grid_layers.end(), "primitives") != grid_layers.end());
  EXPECT_TRUE(std::find(grid_layers.begin(), grid_layers.end(), "combined") != grid_layers.end());
}

TEST_F(TestCostmapGenerator, testLoadAreasFromLaneletMap)
{
  EXPECT_EQ(test_load_road_areas(), 12ul);
  EXPECT_EQ(test_load_parking_areas(), 10ul);
}

TEST_F(TestCostmapGenerator, testIsActive)
{
  EXPECT_TRUE(test_is_active_by_pos());
  EXPECT_FALSE(test_is_active_by_pos(false));
  EXPECT_FALSE(test_is_active_by_pos(true, true));

  EXPECT_TRUE(test_is_active_by_scenario());
  EXPECT_FALSE(test_is_active_by_scenario(false));
  EXPECT_FALSE(test_is_active_by_scenario(true, true));
}

TEST_F(TestCostmapGenerator, PublishesCombinedMapObjectPointLayers)
{
  test_publish_combined_costmap();
}
