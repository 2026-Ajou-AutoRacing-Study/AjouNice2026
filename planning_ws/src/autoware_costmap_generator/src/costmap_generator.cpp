// Copyright 2020-2024 Tier IV, Inc.
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

/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, private_node
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    private_node list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    private_node software without specific prior written permission.
 *
 *  private_node SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF private_node SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************/

#include "autoware/costmap_generator/costmap_generator.hpp"

#include "autoware/costmap_generator/utils/object_map_utils.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <pcl_ros/transforms.h>
#include <boost/make_shared.hpp>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <memory>
#include <string>
#include <vector>

namespace
{

// Copied from scenario selector
geometry_msgs::PoseStamped::ConstPtr getCurrentPose(
  const tf2_ros::Buffer & tf_buffer)
{
  geometry_msgs::TransformStamped tf_current_pose;

  try {
    tf_current_pose = tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
  } catch (tf2::TransformException & ex) {
    ROS_ERROR_THROTTLE(5.0, "%s", ex.what());
    return nullptr;
  }

  geometry_msgs::PoseStamped::Ptr p(new geometry_msgs::PoseStamped());
  p->header = tf_current_pose.header;
  p->pose.orientation = tf_current_pose.transform.rotation;
  p->pose.position.x = tf_current_pose.transform.translation.x;
  p->pose.position.y = tf_current_pose.transform.translation.y;
  p->pose.position.z = tf_current_pose.transform.translation.z;

  return geometry_msgs::PoseStamped::ConstPtr(p);
}

// copied from scenario selector
std::shared_ptr<lanelet::ConstPolygon3d> findNearestParkinglot(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const lanelet::BasicPoint2d & current_position)
{
  auto linked_parking_lot = std::make_shared<lanelet::ConstPolygon3d>();
  const auto result = lanelet::utils::query::getLinkedParkingLot(
    current_position, lanelet_map_ptr, linked_parking_lot.get());

  if (result) {
    return linked_parking_lot;
  }
  return {};
}

// copied from scenario selector
bool isInParkingLot(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const geometry_msgs::Pose & current_pose)
{
  const auto & p = current_pose.position;
  const lanelet::Point3d search_point(lanelet::InvalId, p.x, p.y, p.z);

  const auto nearest_parking_lot =
    findNearestParkinglot(lanelet_map_ptr, search_point.basicPoint2d());

  if (!nearest_parking_lot) {
    return false;
  }

  return lanelet::geometry::within(search_point, nearest_parking_lot->basicPolygon());
}

pcl::PointCloud<pcl::PointXYZ> getTransformedPointCloud(
  const sensor_msgs::PointCloud2 & pointcloud_msg,
  const geometry_msgs::Transform & transform)
{
  const Eigen::Matrix4f transform_matrix = tf2::transformToEigen(transform).matrix().cast<float>();

  sensor_msgs::PointCloud2 transformed_msg;
  pcl_ros::transformPointCloud(transform_matrix, pointcloud_msg, transformed_msg);

  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl::fromROSMsg(transformed_msg, transformed_pointcloud);

  return transformed_pointcloud;
}

std::vector<geometry_msgs::Polygon> getTransformedPrimitives(
  const std::vector<geometry_msgs::Polygon> & in_polygons,
  const geometry_msgs::TransformStamped & transform)
{
  std::vector<geometry_msgs::Polygon> out_polygons;
  for (const auto & polygon : in_polygons) {
    geometry_msgs::Polygon transformed_polygon;
    for (const auto & p : polygon.points) {
      geometry_msgs::Point input, output; input.x = p.x; input.y = p.y; input.z = p.z;
      tf2::doTransform(input, output, transform);
      geometry_msgs::Point32 point; point.x = output.x; point.y = output.y; point.z = output.z;
      transformed_polygon.points.push_back(point);
    }
    out_polygons.emplace_back(transformed_polygon);
  }
  return out_polygons;
}

}  // namespace

namespace autoware::costmap_generator
{
CostmapGenerator::CostmapGenerator(const ros::NodeHandle & private_node)
: pnh_(private_node), tf_listener_(tf_buffer_)
{
  param_ = std::make_shared<::costmap_generator_node::Params>();
  if (!pnh_.getParam("update_rate", param_->update_rate)) throw std::runtime_error("Missing costmap parameter: update_rate");
  if (!pnh_.getParam("activate_by_scenario", param_->activate_by_scenario)) throw std::runtime_error("Missing costmap parameter: activate_by_scenario");
  if (!pnh_.getParam("use_objects", param_->use_objects)) throw std::runtime_error("Missing costmap parameter: use_objects");
  if (!pnh_.getParam("use_points", param_->use_points)) throw std::runtime_error("Missing costmap parameter: use_points");
  if (!pnh_.getParam("use_wayarea", param_->use_wayarea)) throw std::runtime_error("Missing costmap parameter: use_wayarea");
  if (!pnh_.getParam("use_parkinglot", param_->use_parkinglot)) throw std::runtime_error("Missing costmap parameter: use_parkinglot");
  if (!pnh_.getParam("costmap_frame", param_->costmap_frame)) throw std::runtime_error("Missing costmap parameter: costmap_frame");
  if (!pnh_.getParam("vehicle_frame", param_->vehicle_frame)) throw std::runtime_error("Missing costmap parameter: vehicle_frame");
  if (!pnh_.getParam("map_frame", param_->map_frame)) throw std::runtime_error("Missing costmap parameter: map_frame");
  if (!pnh_.getParam("grid_min_value", param_->grid_min_value)) throw std::runtime_error("Missing costmap parameter: grid_min_value");
  if (!pnh_.getParam("grid_max_value", param_->grid_max_value)) throw std::runtime_error("Missing costmap parameter: grid_max_value");
  if (!pnh_.getParam("grid_resolution", param_->grid_resolution)) throw std::runtime_error("Missing costmap parameter: grid_resolution");
  if (!pnh_.getParam("grid_length_x", param_->grid_length_x)) throw std::runtime_error("Missing costmap parameter: grid_length_x");
  if (!pnh_.getParam("grid_length_y", param_->grid_length_y)) throw std::runtime_error("Missing costmap parameter: grid_length_y");
  if (!pnh_.getParam("grid_position_x", param_->grid_position_x)) throw std::runtime_error("Missing costmap parameter: grid_position_x");
  if (!pnh_.getParam("grid_position_y", param_->grid_position_y)) throw std::runtime_error("Missing costmap parameter: grid_position_y");
  if (!pnh_.getParam("maximum_lidar_height_thres", param_->maximum_lidar_height_thres)) throw std::runtime_error("Missing costmap parameter: maximum_lidar_height_thres");
  if (!pnh_.getParam("minimum_lidar_height_thres", param_->minimum_lidar_height_thres)) throw std::runtime_error("Missing costmap parameter: minimum_lidar_height_thres");
  if (!pnh_.getParam("expand_polygon_size", param_->expand_polygon_size)) throw std::runtime_error("Missing costmap parameter: expand_polygon_size");
  if (!pnh_.getParam("size_of_expansion_kernel", param_->size_of_expansion_kernel)) throw std::runtime_error("Missing costmap parameter: size_of_expansion_kernel");
  if (param_->update_rate <= 0) throw std::invalid_argument("Costmap update_rate must be positive");
  sub_lanelet_bin_map_ = pnh_.subscribe("input/vector_map", 1, &CostmapGenerator::onLaneletMapBin, this);
  sub_points_ = pnh_.subscribe<sensor_msgs::PointCloud2>("input/points_no_ground", 1,
    [this](const sensor_msgs::PointCloud2::ConstPtr & msg) {
      std::lock_guard<std::recursive_mutex> lock(callback_mutex_); pending_points_ = msg;
    });
  sub_objects_ = pnh_.subscribe<PredictedObjects>("input/objects", 1,
    [this](const PredictedObjects::ConstPtr & msg) {
      std::lock_guard<std::recursive_mutex> lock(callback_mutex_); pending_objects_ = msg;
    });
  sub_scenario_ = pnh_.subscribe<autoware_internal_planning_msgs::Scenario>("input/scenario", 1,
    [this](const autoware_internal_planning_msgs::Scenario::ConstPtr & msg) {
      std::lock_guard<std::recursive_mutex> lock(callback_mutex_); pending_scenario_ = msg;
    });
  pub_costmap_ = pnh_.advertise<grid_map_msgs::GridMap>("output/grid_map", 1);
  pub_occupancy_grid_ = pnh_.advertise<nav_msgs::OccupancyGrid>("output/occupancy_grid", 1);
  ros::NodeHandle public_node;
  pub_processing_time_ = public_node.advertise<autoware_utils::ProcessingTimeDetail>("processing_time", 1);
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(pub_processing_time_);
  pub_processing_time_ms_ = pnh_.advertise<autoware_internal_debug_msgs::Float64Stamped>("debug/processing_time_ms", 1);
  initGridmap();
  timer_ = pnh_.createTimer(ros::Duration(1.0 / param_->update_rate),
    [this](const ros::TimerEvent &) { onTimer(); });
}

void CostmapGenerator::loadRoadAreasFromLaneletMap(
  const lanelet::LaneletMapPtr lanelet_map,
  std::vector<geometry_msgs::Polygon> & area_polygons)
{
  // use all lanelets in map of subtype road to give way area
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map);
  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);

  // convert lanelets to polygons and put into area_points array
  for (const auto & ll : road_lanelets) {
    geometry_msgs::Polygon poly;
    for (const auto & p : ll.polygon3d().basicPolygon()) {
      const auto pt = experimental::lanelet2_utils::to_ros(p);
      geometry_msgs::Point32 point; point.x = pt.x; point.y = pt.y; point.z = pt.z;
      poly.points.push_back(point);
    }
    area_polygons.push_back(poly);
  }
}

void CostmapGenerator::loadParkingAreasFromLaneletMap(
  const lanelet::LaneletMapPtr lanelet_map,
  std::vector<geometry_msgs::Polygon> & area_polygons)
{
  // Parking lots
  auto toGeomMsgPoly =
    [](const lanelet::ConstPolygon3d & ll_poly, geometry_msgs::Polygon * geom_poly) {
      geom_poly->points.clear();
      geom_poly->points.reserve(ll_poly.size());
      for (const auto & ll_pt : ll_poly) {
        const auto geom_pt = experimental::lanelet2_utils::to_ros(ll_pt.basicPoint());
        geometry_msgs::Point32 point; point.x = geom_pt.x; point.y = geom_pt.y; point.z = geom_pt.z;
        geom_poly->points.push_back(point);
      }
    };

  lanelet::ConstPolygons3d all_parking_lots = lanelet::utils::query::getAllParkingLots(lanelet_map);
  for (const auto & ll_poly : all_parking_lots) {
    geometry_msgs::Polygon poly;
    toGeomMsgPoly(ll_poly, &poly);
    area_polygons.push_back(poly);
  }

  // Parking spaces
  lanelet::ConstLineStrings3d all_parking_spaces =
    lanelet::utils::query::getAllParkingSpaces(lanelet_map);
  for (const auto & parking_space : all_parking_spaces) {
    lanelet::ConstPolygon3d ll_poly;
    lanelet::utils::lineStringWithWidthToPolygon(parking_space, &ll_poly);
    geometry_msgs::Polygon poly;
    toGeomMsgPoly(ll_poly, &poly);
    area_polygons.push_back(poly);
  }
}

void CostmapGenerator::onLaneletMapBin(
  const autoware_map_msgs::LaneletMapBin::ConstPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  lanelet_map_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));

  if (param_->use_wayarea) {
    loadRoadAreasFromLaneletMap(lanelet_map_, primitives_polygons_);
  }

  if (param_->use_parkinglot) {
    loadParkingAreasFromLaneletMap(lanelet_map_, primitives_polygons_);
  }
}

void CostmapGenerator::update_data()
{
  objects_ = pending_objects_;
  points_ = pending_points_;
  scenario_ = pending_scenario_;
}

void CostmapGenerator::set_current_pose()
{
  current_pose_ = getCurrentPose(tf_buffer_);
}

void CostmapGenerator::onTimer()
{
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  update_data();

  autoware_utils::ScopedTimeTrack scoped_time_track(__func__, *time_keeper_);
  stop_watch.tic();

  if (!param_->activate_by_scenario) set_current_pose();

  if (!isActive()) {
    // Publish ProcessingTime
    autoware_internal_debug_msgs::Float64Stamped processing_time_msg;
    const auto stamp = ros::Time::now();
    processing_time_msg.stamp.sec = stamp.sec;
    processing_time_msg.stamp.nanosec = stamp.nsec;
    processing_time_msg.data = stop_watch.toc();
    pub_processing_time_ms_.publish(processing_time_msg);
    return;
  }

  // Get current pose
  time_keeper_->start_track("lookupTransform");
  geometry_msgs::TransformStamped tf;
  try {
    tf =
      tf_buffer_.lookupTransform(param_->costmap_frame, param_->vehicle_frame, ros::Time(0));
  } catch (tf2::TransformException & ex) {
    ROS_ERROR( "%s", ex.what());
    return;
  }
  time_keeper_->end_track("lookupTransform");

  set_grid_center(tf);

  if ((param_->use_wayarea || param_->use_parkinglot) && lanelet_map_) {
    autoware_utils::ScopedTimeTrack st("generatePrimitivesCostmap()", *time_keeper_);
    costmap_[LayerName::primitives] = generatePrimitivesCostmap();
  }

  if (param_->use_objects && objects_) {
    autoware_utils::ScopedTimeTrack st("generateObjectsCostmap()", *time_keeper_);
    costmap_[LayerName::objects] = generateObjectsCostmap(objects_);
  }

  if (param_->use_points && points_) {
    autoware_utils::ScopedTimeTrack st("generatePointsCostmap()", *time_keeper_);
    costmap_[LayerName::points] = generatePointsCostmap(points_, tf.transform.translation.z);
  }

  {
    autoware_utils::ScopedTimeTrack st("generateCombinedCostmap()", *time_keeper_);
    costmap_[LayerName::combined] = generateCombinedCostmap();
  }

  publishCostmap(costmap_, tf);
}

void CostmapGenerator::set_grid_center(const geometry_msgs::TransformStamped & tf)
{
  const auto cur_pos = costmap_.getPosition();
  const grid_map::Position ref_pos(tf.transform.translation.x, tf.transform.translation.y);
  const auto disp = ref_pos - cur_pos;
  const auto resolution = costmap_.getResolution();
  const grid_map::Position offset(
    std::round(disp.x() / resolution) * resolution, std::round(disp.y() / resolution) * resolution);
  costmap_.setPosition(cur_pos + offset);
}

bool CostmapGenerator::isActive()
{
  if (!lanelet_map_) {
    return false;
  }

  if (param_->activate_by_scenario) {
    if (!scenario_) return false;
    const auto & s = scenario_->activating_scenarios;
    return std::any_of(s.begin(), s.end(), [](const auto scenario) {
      return scenario == autoware_internal_planning_msgs::Scenario::PARKING;
    });
  }

  if (!current_pose_) return false;
  return isInParkingLot(lanelet_map_, current_pose_->pose);
}

void CostmapGenerator::initGridmap()
{
  costmap_.setFrameId(param_->costmap_frame);
  costmap_.setGeometry(
    grid_map::Length(param_->grid_length_x, param_->grid_length_y), param_->grid_resolution,
    grid_map::Position(param_->grid_position_x, param_->grid_position_y));

  costmap_.add(LayerName::points, param_->grid_min_value);
  costmap_.add(LayerName::objects, param_->grid_min_value);
  costmap_.add(LayerName::primitives, param_->grid_min_value);
  costmap_.add(LayerName::combined, param_->grid_min_value);
}

grid_map::Matrix CostmapGenerator::generatePointsCostmap(
  const sensor_msgs::PointCloud2::ConstPtr & in_points, const double vehicle_to_map_z)
{
  geometry_msgs::TransformStamped points2costmap;
  try {
    points2costmap = tf_buffer_.lookupTransform(
      param_->costmap_frame, in_points->header.frame_id, ros::Time(0));
  } catch (const tf2::TransformException & ex) {
    ROS_ERROR_THROTTLE(1.0, "%s", ex.what());
  }

  const auto transformed_points = getTransformedPointCloud(*in_points, points2costmap.transform);

  const auto maximum_height_thres = param_->maximum_lidar_height_thres + vehicle_to_map_z;
  const auto minimum_height_thres = param_->minimum_lidar_height_thres + vehicle_to_map_z;
  grid_map::Matrix points_costmap = points2costmap_.makeCostmapFromPoints(
    maximum_height_thres, minimum_height_thres, param_->grid_min_value, param_->grid_max_value,
    costmap_, LayerName::points, transformed_points);

  return points_costmap;
}

PredictedObjects::ConstPtr transformObjects(
  const tf2_ros::Buffer & tf_buffer, const PredictedObjects::ConstPtr in_objects,
  const std::string & target_frame_id, const std::string & src_frame_id)
{
  auto objects = boost::make_shared<PredictedObjects>();
  *objects = *in_objects;
  objects->header.frame_id = target_frame_id;

  geometry_msgs::TransformStamped objects2costmap;
  try {
    objects2costmap = tf_buffer.lookupTransform(target_frame_id, src_frame_id, ros::Time(0));
  } catch (const tf2::TransformException & ex) {
    ROS_ERROR( "%s", ex.what());
  }

  for (auto & object : objects->objects) {
    geometry_msgs::PoseStamped output_stamped;
    geometry_msgs::PoseStamped input_stamped;
    input_stamped.pose = object.kinematics.initial_pose_with_covariance.pose;
    tf2::doTransform(input_stamped, output_stamped, objects2costmap);
    object.kinematics.initial_pose_with_covariance.pose = output_stamped.pose;
  }

  return PredictedObjects::ConstPtr(objects);
}

grid_map::Matrix CostmapGenerator::generateObjectsCostmap(
  const PredictedObjects::ConstPtr in_objects)
{
  const auto object_frame = in_objects->header.frame_id;
  const auto transformed_objects =
    transformObjects(tf_buffer_, in_objects, param_->costmap_frame, object_frame);

  grid_map::Matrix objects_costmap = objects2costmap_.makeCostmapFromObjects(
    costmap_, param_->expand_polygon_size, param_->size_of_expansion_kernel, transformed_objects);

  return objects_costmap;
}

grid_map::Matrix CostmapGenerator::generatePrimitivesCostmap()
{
  grid_map::GridMap lanelet2_costmap = costmap_;
  if (primitives_polygons_.empty()) {
    return lanelet2_costmap[LayerName::primitives];
  }

  geometry_msgs::TransformStamped primitives2costmap;
  try {
    primitives2costmap =
      tf_buffer_.lookupTransform(param_->costmap_frame, param_->map_frame, ros::Time(0));
  } catch (const tf2::TransformException & ex) {
    ROS_ERROR( "%s", ex.what());
  }

  const auto transformed_primitives =
    getTransformedPrimitives(primitives_polygons_, primitives2costmap);

  object_map::fill_polygon_areas(
    lanelet2_costmap, transformed_primitives, LayerName::primitives, param_->grid_max_value,
    param_->grid_min_value);

  return lanelet2_costmap[LayerName::primitives];
}

grid_map::Matrix CostmapGenerator::generateCombinedCostmap()
{
  // assuming combined_costmap is calculated by element wise max operation
  grid_map::GridMap combined_costmap = costmap_;

  combined_costmap[LayerName::combined].setConstant(param_->grid_min_value);

  combined_costmap[LayerName::combined] =
    combined_costmap[LayerName::combined].cwiseMax(combined_costmap[LayerName::points]);

  combined_costmap[LayerName::combined] =
    combined_costmap[LayerName::combined].cwiseMax(combined_costmap[LayerName::primitives]);

  combined_costmap[LayerName::combined] =
    combined_costmap[LayerName::combined].cwiseMax(combined_costmap[LayerName::objects]);

  return combined_costmap[LayerName::combined];
}

void CostmapGenerator::publishCostmap(
  const grid_map::GridMap & costmap, const geometry_msgs::TransformStamped & tf)
{
  // Set header
  std_msgs::Header header;
  header.frame_id = param_->costmap_frame;
  header.stamp = ros::Time::now();

  // Publish OccupancyGrid
  nav_msgs::OccupancyGrid out_occupancy_grid;
  grid_map::GridMapRosConverter::toOccupancyGrid(
    costmap, LayerName::combined, param_->grid_min_value, param_->grid_max_value,
    out_occupancy_grid);
  out_occupancy_grid.header = header;
  out_occupancy_grid.info.origin.position.z = tf.transform.translation.z;
  pub_occupancy_grid_.publish(out_occupancy_grid);

  // Publish GridMap
  grid_map_msgs::GridMap out_gridmap_msg;
  grid_map::GridMapRosConverter::toMessage(costmap, out_gridmap_msg);
  out_gridmap_msg.info.header = header;
  out_gridmap_msg.info.pose.position.z = tf.transform.translation.z;
  pub_costmap_.publish(out_gridmap_msg);

  // Publish ProcessingTime
  autoware_internal_debug_msgs::Float64Stamped processing_time_msg;
  const auto stamp = ros::Time::now();
    processing_time_msg.stamp.sec = stamp.sec;
    processing_time_msg.stamp.nanosec = stamp.nsec;
  processing_time_msg.data = stop_watch.toc();
  pub_processing_time_ms_.publish(processing_time_msg);
}
}  // namespace autoware::costmap_generator

