#include "autoware_path_sampler/node.hpp"
#include "autoware_path_sampler/path_generation.hpp"
#include "autoware_path_sampler/prepare_inputs.hpp"

#include <autoware_sampler_common/constraints/hard_constraint.hpp>
#include <autoware_sampler_common/constraints/soft_constraint.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace
{
void setRequiredParameters(ros::NodeHandle & node)
{
  node.setParam("wheel_radius", 0.39);
  node.setParam("wheel_width", 0.42);
  node.setParam("wheel_base", 2.79);
  node.setParam("wheel_tread", 1.63);
  node.setParam("front_overhang", 1.0);
  node.setParam("rear_overhang", 1.0);
  node.setParam("left_overhang", 0.2);
  node.setParam("right_overhang", 0.2);
  node.setParam("vehicle_height", 1.5);
  node.setParam("max_steer_angle", 0.7);
  node.setParam("common/output_delta_arc_length", 0.5);
  node.setParam("debug/enable_calculation_time_info", false);
  node.setParam("debug/id", 0);
  node.setParam("preprocessing/force_zero_initial_deviation", false);
  node.setParam("preprocessing/force_zero_initial_heading", false);
  node.setParam("preprocessing/smooth_reference_trajectory", false);
  node.setParam("constraints/hard/max_curvature", 1.0);
  node.setParam("constraints/hard/min_curvature", -1.0);
  node.setParam("constraints/hard/min_distance_from_obstacles", 0.1);
  node.setParam("constraints/hard/limit_footprint_inside_drivable_area", false);
  node.setParam("constraints/soft/lateral_deviation_weight", 1.0);
  node.setParam("constraints/soft/length_weight", 1.0);
  node.setParam("constraints/soft/curvature_weight", 1.0);
  node.setParam("path_reuse/maximum_lateral_deviation", 1.0);
  node.setParam("path_reuse/direct_reuse_distance", 2.0);
  node.setParam("sampling/enable_frenet", true);
  node.setParam("sampling/enable_bezier", true);
  node.setParam("sampling/resolution", 0.5);
  node.setParam("sampling/previous_path_reuse_points_nb", 2);
  node.setParam("sampling/target_lengths", std::vector<double>{10.0, 20.0});
  node.setParam("sampling/target_lateral_positions", std::vector<double>{0.0});
  node.setParam("sampling/nb_target_lateral_positions", 0);
  node.setParam("sampling/frenet/target_lateral_velocities", std::vector<double>{0.0});
  node.setParam("sampling/frenet/target_lateral_accelerations", std::vector<double>{0.0});
  node.setParam("sampling/bezier/nb_k", 3);
  node.setParam("sampling/bezier/mk_min", 0.0);
  node.setParam("sampling/bezier/mk_max", 10.0);
  node.setParam("sampling/bezier/nb_t", 5);
  node.setParam("sampling/bezier/mt_min", 0.3);
  node.setParam("sampling/bezier/mt_max", 1.7);
  node.setParam("ego_nearest_dist_threshold", 3.0);
  node.setParam("ego_nearest_yaw_threshold", 1.57);
}

autoware_planning_msgs::Path makePath()
{
  autoware_planning_msgs::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i < 25; ++i) {
    autoware_planning_msgs::PathPoint point;
    point.pose.position.x = 5.0 * i;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 5.0F;
    path.points.push_back(point);
    geometry_msgs::Point left;
    left.x = 5.0 * i;
    left.y = 5.0;
    path.left_bound.push_back(left);
    geometry_msgs::Point right = left;
    right.y = -5.0;
    path.right_bound.push_back(right);
  }
  return path;
}

void verifyCandidateGeneration()
{
  const auto path = makePath();
  std::vector<autoware_planning_msgs::TrajectoryPoint> reference;
  for (const auto & point : path.points) {
    autoware_planning_msgs::TrajectoryPoint trajectory_point;
    trajectory_point.pose = point.pose;
    trajectory_point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
    reference.push_back(trajectory_point);
  }
  auto params = Parameters{};
  params.sampling.enable_frenet = true;
  params.sampling.enable_bezier = true;
  params.sampling.resolution = 0.5;
  params.sampling.target_lengths = {10.0, 20.0};
  params.sampling.target_lateral_positions = {0.0};
  params.sampling.nb_target_lateral_positions = 0;
  params.sampling.frenet.target_lateral_velocities = {0.0};
  params.sampling.frenet.target_lateral_accelerations = {0.0};
  params.sampling.bezier.nb_k = 3;
  params.sampling.bezier.mk_min = 0.0;
  params.sampling.bezier.mk_max = 10.0;
  params.sampling.bezier.nb_t = 5;
  params.sampling.bezier.mt_min = 0.3;
  params.sampling.bezier.mt_max = 1.7;
  params.constraints.hard.min_curvature = -1.0;
  params.constraints.hard.max_curvature = 1.0;
  params.constraints.hard.min_dist_from_obstacles = 0.1;
  params.constraints.hard.limit_footprint_inside_drivable_area = false;
  params.constraints.soft.lateral_deviation_weight = 1.0;
  params.constraints.soft.length_weight = 1.0;
  params.constraints.soft.curvature_weight = 1.0;
  const auto info = autoware::vehicle_info_utils::createVehicleInfo(
    0.39, 0.42, 2.79, 1.63, 1.0, 1.0, 0.2, 0.2, 1.5, 0.7);
  params.constraints.ego_footprint = info.createFootprint();
  params.constraints.ego_width = info.vehicle_width_m;
  params.constraints.ego_length = info.vehicle_length_m;
  autoware::path_sampler::prepareConstraints(
    params.constraints, autoware_perception_msgs::PredictedObjects{}, path.left_bound, path.right_bound);
  const auto spline = autoware::path_sampler::preparePathSpline(reference, false);
  autoware::sampler_common::State initial;
  initial.pose = {0.0, 0.0};
  initial.frenet = spline.frenet(initial.pose);
  initial.heading = 0.0;
  initial.curvature = spline.curvature(initial.frenet.s);
  auto candidates = autoware::path_sampler::generateCandidatePaths(initial, spline, 0.0, params);
  size_t valid_finite = 0;
  size_t finite_cost = 0;
  size_t nonempty_curvature = 0;
  for (auto & candidate : candidates) {
    autoware::sampler_common::constraints::checkHardConstraints(candidate, params.constraints);
    autoware::sampler_common::constraints::calculateCost(candidate, params.constraints, spline);
    if (std::isfinite(candidate.cost)) ++finite_cost;
    if (!candidate.curvatures.empty()) ++nonempty_curvature;
    if (candidate.constraint_results.isValid() && std::isfinite(candidate.cost)) ++valid_finite;
  }
  if (valid_finite == 0) {
    ROS_ERROR_STREAM(
      "candidate diagnostic count=" << candidates.size() << " finite=" << finite_cost
      << " curvature=" << nonempty_curvature);
    throw std::runtime_error("no valid finite path-sampler candidate");
  }
  ROS_INFO_STREAM("path sampler candidate probe valid_finite=" << valid_finite);
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "path_sampler_runtime_probe");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  setRequiredParameters(private_node);
  verifyCandidateGeneration();
  autoware::path_sampler::PathSampler sampler(node, private_node);

  std::atomic<size_t> output_points{0};
  std::atomic<size_t> output_count{0};
  std::atomic<double> first_velocity{0.0};
  const auto output_sub = private_node.subscribe<autoware_planning_msgs::Trajectory>(
    "output/path", 1, [&output_points, &output_count, &first_velocity](const autoware_planning_msgs::Trajectory::ConstPtr & msg) {
      output_points.store(msg->points.size());
      output_count.fetch_add(1);
      if (!msg->points.empty()) first_velocity.store(msg->points.front().longitudinal_velocity_mps);
    });
  const auto odom_pub = private_node.advertise<nav_msgs::Odometry>("input/odometry", 1, true);
  const auto objects_pub =
    private_node.advertise<autoware_perception_msgs::PredictedObjects>("input/objects", 1, true);
  const auto path_pub = private_node.advertise<autoware_planning_msgs::Path>("input/path", 1, false);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while ((path_pub.getNumSubscribers() == 0 || odom_pub.getNumSubscribers() == 0 ||
          objects_pub.getNumSubscribers() == 0) && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (path_pub.getNumSubscribers() == 0 || odom_pub.getNumSubscribers() == 0 ||
      objects_pub.getNumSubscribers() == 0) {
    throw std::runtime_error("path sampler subscriptions did not connect");
  }

  nav_msgs::Odometry odom;
  odom.pose.pose.orientation.w = 1.0;
  odom.twist.twist.linear.x = 5.0;
  odom_pub.publish(odom);
  objects_pub.publish(autoware_perception_msgs::PredictedObjects{});
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  path_pub.publish(makePath());

  while (output_points.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  odom.pose.pose.position.x = 1.0;
  odom_pub.publish(odom);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  path_pub.publish(makePath());
  while (output_count.load() < 2 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  spinner.stop();
  if (output_points.load() <= makePath().points.size() || output_count.load() < 2) {
    std::cerr << "runtime output points=" << output_points.load()
              << " outputs=" << output_count.load()
              << " first_velocity=" << first_velocity.load() << std::endl;
    throw std::runtime_error("path sampler did not select a resampled candidate trajectory");
  }
  ROS_INFO_STREAM(
    "path sampler runtime probe output_points=" << output_points.load()
    << " reuse_outputs=" << output_count.load()
    << " first_velocity=" << first_velocity.load());
  return 0;
}
