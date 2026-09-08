#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_map_msgs/LaneletMapBin.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <lanelet2_core/LaneletMap.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace
{
autoware_planning_msgs::LaneletRoute route_for(const lanelet::ConstLanelet & lane)
{
  autoware_planning_msgs::LaneletPrimitive p; p.id = lane.id(); p.primitive_type = "lanelet";
  autoware_planning_msgs::LaneletSegment s; s.preferred_primitive = p; s.primitives.push_back(p);
  autoware_planning_msgs::LaneletRoute route; route.segments.push_back(s); return route;
}
bool received = false;
bool received_source_shaped_diagnostics = false;
bool received_any_diagnostics = false;
bool received_interval_error = false;
bool received_finite_error = false;
bool received_finite_warn = false;
int output_count = 0;
autoware_planning_msgs::Trajectory last_output;
void output(const autoware_planning_msgs::TrajectoryConstPtr & msg)
{
  received = msg && msg->points.size() >= 2U;
  if (received) { ++output_count; last_output = *msg; }
}
void diagnostics(const diagnostic_msgs::DiagnosticArrayConstPtr & msg)
{
  received_any_diagnostics = msg != nullptr;
  if (!msg || msg->status.size() != 19U) return;
  const auto finite = std::find_if(msg->status.begin(), msg->status.end(), [](const auto & status) {
    return status.name == "trajectory_validation_finite";
  });
  const auto rear = std::find_if(msg->status.begin(), msg->status.end(), [](const auto & status) {
    return status.name == "rear_collision_check";
  });
  received_source_shaped_diagnostics = finite != msg->status.end() && rear != msg->status.end() &&
    finite->level == diagnostic_msgs::DiagnosticStatus::OK && rear->level == diagnostic_msgs::DiagnosticStatus::OK;
  const auto interval = std::find_if(msg->status.begin(), msg->status.end(), [](const auto & status) {
    return status.name == "trajectory_validation_interval";
  });
  received_interval_error = received_interval_error || (interval != msg->status.end() &&
    interval->level == diagnostic_msgs::DiagnosticStatus::ERROR &&
    interval->message == "points interval is too large");
  received_finite_error = received_finite_error || (finite != msg->status.end() &&
    finite->level == diagnostic_msgs::DiagnosticStatus::ERROR &&
    finite->message == "infinite value is found");
  received_finite_warn = received_finite_warn || (finite != msg->status.end() &&
    finite->level == diagnostic_msgs::DiagnosticStatus::WARN &&
    finite->message == "infinite value is found (invalid count is less than error threshold: 1 < 2)");
}
}
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "planning_validator_node_interface_oracle"); ros::NodeHandle nh;
  const std::string mode = argc == 2 ? std::string(argv[1]) : std::string{};
  const bool expect_no_diagnostics = mode == "--expect-no-diagnostics";
  const bool exercise_soft_stop = mode == "--exercise-soft-stop";
  const bool exercise_interval_diagnostic = mode == "--exercise-interval-diagnostic";
  const bool exercise_finite_diagnostic = mode == "--exercise-finite-diagnostic";
  const bool exercise_diagnostic_threshold = mode == "--exercise-diagnostic-threshold";
  // This mode publishes only the graph's declared external ingress and the
  // MVP input.  It therefore verifies MVP output feeding the Validator rather
  // than bypassing the graph through the standalone private topics.
  const bool exercise_pipeline_remapped = mode == "--exercise-pipeline-remapped";
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(AW149_ROUTE_FIXTURE_PATH);
  if (!map || map->laneletLayer.empty()) return 2;
  // The overlap-map fixture contains short connector lanelets.  Select the
  // longest centerline so the source forward-trajectory-length gate can pass
  // before this oracle intentionally introduces a lateral shift.
  const auto lane = *std::max_element(
    map->laneletLayer.begin(), map->laneletLayer.end(), [](const auto & left, const auto & right) {
      const auto length = [](const auto & lanelet) {
        double total = 0.0;
        const auto centerline = lanelet.centerline();
        for (size_t index = 1; index < centerline.size(); ++index) {
          total += std::hypot(
            centerline[index].x() - centerline[index - 1U].x(),
            centerline[index].y() - centerline[index - 1U].y());
        }
        return total;
      };
      return length(left) < length(right);
    });
  auto bin = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  bin.version_map_format = "2.0";
  auto route = route_for(lane);
  autoware_planning_msgs::Trajectory trajectory;
  for (const auto & c : lane.centerline()) { autoware_planning_msgs::TrajectoryPoint p; p.pose.position.x=c.x(); p.pose.position.y=c.y(); p.pose.position.z=c.z(); p.pose.orientation.w=1.0; trajectory.points.push_back(p); }
  if (trajectory.points.size() < 2U) return 3;
  // Keep the short fixture's normal baseline inside the source forward-length
  // gate. The positive-speed soft-stop profile is covered by the dedicated
  // core oracle; this node oracle verifies that an invalid trajectory shift
  // selects the source-configured policy 2 over the default policy 0.
  for (auto & point : trajectory.points) point.longitudinal_velocity_mps = 0.0F;
  nav_msgs::Odometry odom; const auto c = lane.centerline()[lane.centerline().size()/2U]; odom.pose.pose.position.x=c.x(); odom.pose.pose.position.y=c.y(); odom.pose.pose.orientation.w=1.0; odom.twist.twist.linear.x = 0.0;
  const auto trajectory_input = exercise_pipeline_remapped ?
    "/planning/scenario_planning/lane_driving/motion_planning/input_trajectory" :
    "/planning_validator_candidate/input/trajectory";
  const auto kinematics_input = exercise_pipeline_remapped ?
    "/localization/kinematic_state" : "/planning_validator_candidate/input/kinematics";
  const auto acceleration_input = exercise_pipeline_remapped ?
    "/localization/acceleration" : "/planning_validator_candidate/input/acceleration";
  const auto pointcloud_input = exercise_pipeline_remapped ?
    "/perception/obstacle_segmentation/pointcloud" : "/planning_validator_candidate/input/pointcloud";
  const auto route_input = exercise_pipeline_remapped ?
    "/planning/mission_planning/route" : "/planning_validator_candidate/input/route";
  const auto map_input = exercise_pipeline_remapped ?
    "/map/vector_map" : "/planning_validator_candidate/input/lanelet_map_bin";
  const auto trajectory_output = exercise_pipeline_remapped ?
    "/planning/trajectory" : "/planning_validator_candidate/output/trajectory";
  const auto diagnostics_output = exercise_pipeline_remapped ?
    "/planning/planning_validator/diagnostics" : "/planning_validator_candidate/diagnostics";
  auto tpub=nh.advertise<autoware_planning_msgs::Trajectory>(trajectory_input,1,true);
  auto opub=nh.advertise<nav_msgs::Odometry>(kinematics_input,1,true);
  auto apub=nh.advertise<geometry_msgs::AccelWithCovarianceStamped>(acceleration_input,1,true);
  auto ppub=nh.advertise<sensor_msgs::PointCloud2>(pointcloud_input,1,true);
  auto rpub=nh.advertise<autoware_planning_msgs::LaneletRoute>(route_input,1,true);
  auto mpub=nh.advertise<autoware_map_msgs::LaneletMapBin>(map_input,1,true);
  auto sub=nh.subscribe(trajectory_output,1,output);
  auto diag_sub=nh.subscribe(diagnostics_output,1,diagnostics);
  geometry_msgs::AccelWithCovarianceStamped acceleration; sensor_msgs::PointCloud2 pointcloud;
  if (exercise_soft_stop) {
    ros::Rate first_rate(20);
    for (int i = 0; i < 40 && output_count == 0; ++i) {
      opub.publish(odom); apub.publish(acceleration); ppub.publish(pointcloud); rpub.publish(route); mpub.publish(bin); tpub.publish(trajectory);
      ros::spinOnce(); first_rate.sleep();
    }
    if (output_count == 0) return 5;
    auto shifted = trajectory;
    for (auto & point : shifted.points) point.pose.position.y += 1.0;
    const int baseline_count = output_count;
    for (int i = 0; i < 40 && output_count == baseline_count; ++i) {
      opub.publish(odom); apub.publish(acceleration); ppub.publish(pointcloud); rpub.publish(route); mpub.publish(bin); tpub.publish(shifted);
      ros::spinOnce(); first_rate.sleep();
    }
    if (output_count == baseline_count || last_output.points.size() != trajectory.points.size()) return 6;
    if (std::abs(last_output.points.front().pose.position.y - trajectory.points.front().pose.position.y) > 1.0e-6) return 7;
    if (last_output.points.back().longitudinal_velocity_mps != 0.0F) return 8;
    return 0;
  }
  if (exercise_interval_diagnostic) {
    // Source DiagCheckInterval treats an interval above the configured
    // threshold as a named ERROR. The candidate YAML threshold is 100 m, so
    // this vector crosses it without changing route/map readiness.
    auto invalid = trajectory;
    invalid.points.back().pose.position.x += 101.0;
    ros::Rate interval_rate(20);
    for (int i = 0; i < 40; ++i) {
      opub.publish(odom); apub.publish(acceleration); ppub.publish(pointcloud); rpub.publish(route); mpub.publish(bin); tpub.publish(invalid);
      ros::spinOnce(); interval_rate.sleep();
      if (received_interval_error) return 0;
    }
    return 9;
  }
  if (exercise_finite_diagnostic) {
    auto invalid = trajectory;
    invalid.points.at(invalid.points.size() / 2U).pose.position.x = std::numeric_limits<double>::quiet_NaN();
    ros::Rate finite_rate(20);
    for (int i = 0; i < 40; ++i) {
      opub.publish(odom); apub.publish(acceleration); ppub.publish(pointcloud); rpub.publish(route); mpub.publish(bin); tpub.publish(invalid);
      ros::spinOnce(); finite_rate.sleep();
      if (received_finite_error) return 0;
    }
    return 10;
  }
  if (exercise_diagnostic_threshold) {
    auto invalid = trajectory;
    invalid.points.at(invalid.points.size() / 2U).pose.position.x = std::numeric_limits<double>::quiet_NaN();
    ros::Rate threshold_rate(20);
    for (int i = 0; i < 40; ++i) {
      opub.publish(odom); apub.publish(acceleration); ppub.publish(pointcloud); rpub.publish(route); mpub.publish(bin); tpub.publish(invalid);
      ros::spinOnce(); threshold_rate.sleep();
      if (received_finite_warn && received_finite_error) return 0;
    }
    return 11;
  }
  ros::Rate rate(20); for(int i=0;i<40;++i){ opub.publish(odom); apub.publish(acceleration); ppub.publish(pointcloud); rpub.publish(route); mpub.publish(bin); tpub.publish(trajectory); ros::spinOnce(); rate.sleep(); if(!expect_no_diagnostics && received && received_source_shaped_diagnostics) return 0; }
  if (expect_no_diagnostics && received && !received_any_diagnostics) return 0;
  return 4;
}
