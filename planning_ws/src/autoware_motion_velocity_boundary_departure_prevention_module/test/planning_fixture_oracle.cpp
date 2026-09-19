#include "boundary_departure_prevention_module.hpp"
#include "utils.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <boost/make_shared.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LineString.h>
#include <ros/ros.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner::experimental
{
class BoundaryDeparturePlanningFixtureOracle
{
public:
  static int run()
  {
    const auto forward_departure = run_fixture(1.2, true);
    if (!forward_departure.has_value()) {
      return fail("BDP_PLANNING_FIXTURE_FORWARD_EXECUTION_FAILED");
    }
    if (!forward_departure->slowdown_intervals.empty()) {
      return fail("BDP_PLANNING_FIXTURE_FORWARD_SOURCE_PARITY_CHANGED");
    }

    const auto departure = run_fixture(1.2, false);
    if (!departure.has_value()) {
      return fail("BDP_PLANNING_FIXTURE_DEPARTURE_EXECUTION_FAILED");
    }
    if (departure->slowdown_intervals.empty()) {
      return fail("BDP_PLANNING_FIXTURE_DEPARTURE_DID_NOT_SLOW_DOWN");
    }

    const auto control = run_fixture(10.0, false);
    if (!control.has_value()) {
      return fail("BDP_PLANNING_FIXTURE_CONTROL_EXECUTION_FAILED");
    }
    if (!control->slowdown_intervals.empty()) {
      return fail("BDP_PLANNING_FIXTURE_NO_DEPARTURE_FALSE_SLOW_DOWN");
    }

    std::cout << "BDP_PLANNING_FIXTURE_FORWARD_SOURCE_PARITY_AND_REVERSE_BOUNDARY_SLOWDOWN_PASS" << std::endl;
    return 0;
  }

private:
  static std::optional<VelocityPlanningResult> run_fixture(
    const double boundary_y, const bool is_driving_forward)
  {
    ros::NodeHandle node;
    BoundaryDeparturePreventionModule module;
    module.init(node, "boundary_departure_planning_fixture_oracle");
    configure(module);
    // Isolate the persistence branch from the just-initialized timer. A
    // boundary already present before this planning cycle must be eligible for
    // interval construction when the configured on-buffer is zero.
    module.last_abnormality_fp_overlap_bound_time_ = ros::Time::now().toSec() - 1.0;

    const auto raw = make_trajectory(0.0);
    auto predicted = autoware_planning_msgs::Trajectory{};
    predicted.header.stamp = ros::Time::now();
    predicted.points = raw;
    module.ego_pred_traj_ptr_ = boost::make_shared<autoware_planning_msgs::Trajectory>(predicted);

    auto control = boost::make_shared<autoware_control_msgs::Control>();
    const auto control_stamp = ros::Time::now();
    control->stamp.sec = control_stamp.sec;
    control->stamp.nanosec = control_stamp.nsec;
    module.control_cmd_ptr_ = control;

    auto steering = boost::make_shared<autoware_vehicle_msgs::SteeringReport>();
    const auto steering_stamp = ros::Time::now();
    steering->stamp.sec = steering_stamp.sec;
    steering->stamp.nanosec = steering_stamp.nsec;
    module.steering_angle_ptr_ = steering;

    auto mode = boost::make_shared<autoware_adapi_v1_msgs::OperationModeState>();
    mode->mode = autoware_adapi_v1_msgs::OperationModeState::AUTONOMOUS;
    mode->is_autoware_control_enabled = true;
    module.op_mode_state_ptr_ = mode;
    module.route_ptr_ = boost::make_shared<autoware_planning_msgs::LaneletRoute>();

    auto planner_data = std::make_shared<PlannerData>();
    planner_data->route_handler = std::make_shared<autoware::route_handler::RouteHandler>(make_map(boundary_y));
    planner_data->vehicle_info_ = make_vehicle();
    planner_data->current_odometry.header.stamp = ros::Time::now();
    planner_data->current_odometry.pose.pose.orientation.w = 1.0;
    planner_data->current_odometry.twist.twist.linear.x = 5.0;
    planner_data->current_acceleration.accel.accel.linear.x = 0.0;
    // The frozen upstream call supplies the direction-dependent ego offset as
    // the interval grouping argument. Preserve both branches: forward remains
    // empty for this fixture, whereas reverse can group the same points.
    planner_data->is_driving_forward = is_driving_forward;

    // The first manager cycle initializes the route snapshot. Exercise the
    // same repeated-call lifecycle as the runtime planner and judge the next
    // cycle, where boundary output is eligible to persist.
    const auto warmup = module.plan(raw, raw, planner_data);
    const auto warmup_departure_intervals = module.output_.departure_intervals.size();
    const auto warmup_slowdown_intervals = module.output_.slowdown_intervals.size();
    const auto result = module.plan(raw, raw, planner_data);
    const auto left_closest = module.output_.abnormalities_data.closest_projections_to_bound.left.size();
    const auto right_closest = module.output_.abnormalities_data.closest_projections_to_bound.right.size();
    const auto left_departure = module.output_.abnormalities_data.departure_points.left.size();
    const auto right_departure = module.output_.abnormalities_data.departure_points.right.size();
    const auto rebuilt_trajectory = trajectory::Trajectory<TrajectoryPoint>::Builder{}.build(raw);
    const auto manual_intervals = rebuilt_trajectory
      ? utils::init_departure_intervals(
          *rebuilt_trajectory, module.output_.abnormalities_data.departure_points,
          planner_data->vehicle_info_.vehicle_length_m, module.node_param_.slow_down_types)
      : DepartureIntervals{};
    auto manual_update_intervals = manual_intervals;
    auto manual_departure_points = module.output_.abnormalities_data.departure_points;
    if (rebuilt_trajectory) {
      utils::update_departure_intervals(
        manual_update_intervals, manual_departure_points, *rebuilt_trajectory,
        planner_data->vehicle_info_.vehicle_length_m, raw,
        -planner_data->vehicle_info_.max_longitudinal_offset_m,
        module.node_param_.th_pt_shift_dist_m, module.node_param_.th_pt_shift_angle_rad,
        module.node_param_.slow_down_types, false, true);
    }
    std::cerr << "BDP_FIXTURE_DIAG boundary_y=" << boundary_y
              << " forward=" << is_driving_forward
              << " warmup_result=" << warmup.slowdown_intervals.size()
              << " warmup_intervals=" << warmup_departure_intervals
              << " warmup_slowdown=" << warmup_slowdown_intervals
              << " closest=" << left_closest << "," << right_closest
              << " departure=" << left_departure << "," << right_departure
              << " intervals=" << module.output_.departure_intervals.size()
              << " manual_intervals=" << manual_intervals.size()
              << " manual_after_update=" << manual_update_intervals.size()
              << " slowdown=" << module.output_.slowdown_intervals.size() << std::endl;
    std::cerr << "BDP_FIXTURE_PERSIST_DIAG elapsed="
              << (ros::Time::now().toSec() - module.last_abnormality_fp_overlap_bound_time_)
              << " threshold=" << module.node_param_.on_time_buffer_s.near_boundary << std::endl;
    if (!manual_intervals.empty()) {
      std::cerr << "BDP_FIXTURE_MANUAL_INTERVAL start=" << manual_intervals.front().start_dist_on_traj
                << " end=" << manual_intervals.front().end_dist_on_traj << std::endl;
    }
    for (const auto & point : module.output_.abnormalities_data.departure_points.left) {
      std::cerr << "BDP_FIXTURE_DEPARTURE_TYPE " << static_cast<int>(point.departure_type)
                << " lateral_distance=" << point.lat_dist_to_bound << std::endl;
    }
    return result;
  }

  static void configure(BoundaryDeparturePreventionModule & module)
  {
    module.node_param_.bdc_param.boundary_types_to_detect = {"road_border"};
    module.node_param_.bdc_param.th_max_lateral_query_num = 5;
    module.node_param_.bdc_param.th_cutoff_time_predicted_path_s = 10.0;
    module.node_param_.bdc_param.th_cutoff_time_near_boundary_s = 10.0;
    module.node_param_.bdc_param.th_cutoff_time_departure_s = 10.0;
    module.node_param_.bdc_param.footprint_types_to_check = {FootprintType::NORMAL};
    module.node_param_.bdc_param.abnormality_configs[FootprintType::NORMAL] = NormalConfig{};
    module.node_param_.bdc_param.th_trigger.th_vel_mps = {1.0, 10.0};
    module.node_param_.bdc_param.th_trigger.th_acc_mps2 = {-1.0, -2.5};
    module.node_param_.bdc_param.th_trigger.th_jerk_mps3 = {-1.0, -1.5};
    module.node_param_.bdc_param.th_trigger.brake_delay_s = 0.0;
    module.node_param_.bdc_param.th_trigger.dist_error_m = 0.0;
    module.node_param_.bdc_param.th_trigger.th_dist_to_boundary_m.left = {0.01, 0.5};
    module.node_param_.bdc_param.th_trigger.th_dist_to_boundary_m.right = {0.01, 0.5};
    // The geometric fixture can classify the same physical boundary as near,
    // approaching, or critical according to vehicle footprint overlap. Enable
    // every source-supported mitigation severity, then assert the boundary
    // case emits an interval while the distant-boundary control does not.
    module.node_param_.slow_down_types = {
      DepartureType::NEAR_BOUNDARY, DepartureType::APPROACHING_DEPARTURE,
      DepartureType::CRITICAL_DEPARTURE};
    module.node_param_.on_time_buffer_s.near_boundary = 0.0;
    module.node_param_.off_time_buffer_s.near_boundary = 0.0;
  }

  static std::vector<autoware_planning_msgs::TrajectoryPoint> make_trajectory(const double y)
  {
    std::vector<autoware_planning_msgs::TrajectoryPoint> points;
    // The source groups neighbouring departure candidates only when their
    // longitudinal spacing is below vehicle length; use 1 m samples so the
    // fixture exercises interval creation rather than its sparse-input guard.
    for (int x = 0; x <= 40; ++x) {
      autoware_planning_msgs::TrajectoryPoint point;
      point.pose.position.x = static_cast<double>(x);
      point.pose.position.y = y;
      point.pose.orientation.w = 1.0;
      point.longitudinal_velocity_mps = 5.0;
      // The overlay preserves ROS2 duration fields under this ROS1-facing
      // message name; populate the serialized fields, not ros::Duration.
      point.time_from_start.sec = x / 5;
      point.time_from_start.nanosec = static_cast<uint32_t>((x % 5) * 200000000);
      points.push_back(point);
    }
    return points;
  }

  static lanelet::LaneletMapPtr make_map(const double boundary_y)
  {
    auto map = std::make_shared<lanelet::LaneletMap>();
    lanelet::Point3d first{101, -10.0, boundary_y, 0.0};
    lanelet::Point3d second{102, 60.0, boundary_y, 0.0};
    lanelet::LineString3d boundary{100, {first, second}};
    boundary.attributes()[lanelet::AttributeName::Type] = "road_border";
    map->add(boundary);
    return map;
  }

  static autoware::vehicle_info_utils::VehicleInfo make_vehicle()
  {
    return autoware::vehicle_info_utils::createVehicleInfo(
      0.3, 0.2, 2.5, 1.5, 1.0, 1.0, 0.25, 0.25, 1.5, 0.5);
  }

  static int fail(const std::string & reason)
  {
    std::cerr << reason << std::endl;
    return 2;
  }
};
}  // namespace autoware::motion_velocity_planner::experimental

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "bdp_planning_fixture_oracle");
  return autoware::motion_velocity_planner::experimental::BoundaryDeparturePlanningFixtureOracle::run();
}
