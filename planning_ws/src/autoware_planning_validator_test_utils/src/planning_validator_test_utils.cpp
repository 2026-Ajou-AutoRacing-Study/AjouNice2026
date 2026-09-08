#include <autoware_planning_validator_test_utils/planning_validator_test_utils.hpp>
#include <autoware_planning_validator_test_utils/test_parameters.hpp>

#include <algorithm>
#include <cmath>

namespace autoware::planning_validator::test_utils {
namespace {
geometry_msgs::Quaternion quaternionFromYaw(const double yaw) {
  geometry_msgs::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}
autoware_planning_msgs::TrajectoryPoint makePoint(const double x, const double y, const double yaw,
                                                   const double v, const double a = 0.0,
                                                   const double steering = 0.0) {
  autoware_planning_msgs::TrajectoryPoint p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation = quaternionFromYaw(yaw);
  p.longitudinal_velocity_mps = v;
  p.acceleration_mps2 = a;
  p.front_wheel_angle_rad = steering;
  return p;
}
Trajectory stampedTrajectory() { Trajectory t; t.header.stamp = ros::Time::now(); return t; }
}  // namespace

Trajectory generateTrajectoryWithConstantAcceleration(const double interval_distance, const double speed,
  const double yaw, const size_t size, const double acceleration) {
  Trajectory t = stampedTrajectory(); double s = 0.0, v = speed, a = acceleration;
  constexpr double max_dt = 10.0;
  for (size_t i = 0; i < size; ++i) {
    t.points.push_back(makePoint(s * std::cos(yaw), s * std::sin(yaw), yaw, v, a));
    s += interval_distance;
    const double dt = std::abs(v) > 0.1 ? interval_distance / v : max_dt;
    v += acceleration * dt;
    if (v < 0.0) { v = 0.0; a = 0.0; }
  }
  return t;
}

Trajectory generateTrajectoryWithVariableAcceleration(const double interval_distance, const double initial_speed,
  const double yaw, const size_t size, const std::function<double(size_t)> & pattern) {
  Trajectory t = stampedTrajectory(); double s = 0.0, v = initial_speed;
  constexpr double max_dt = 10.0;
  for (size_t i = 0; i < size; ++i) {
    const double a = pattern(i);
    t.points.push_back(makePoint(s * std::cos(yaw), s * std::sin(yaw), yaw, v, a));
    s += interval_distance;
    v += a * (std::abs(v) > 0.1 ? interval_distance / v : max_dt);
    if (v < 0.0) v = 0.0;
  }
  return t;
}

Trajectory generateTrajectoryWithSinusoidalAcceleration(const double interval_distance, const double initial_speed,
  const double yaw, const size_t size, const double max_acceleration, const double period) {
  return generateTrajectoryWithVariableAcceleration(interval_distance, initial_speed, yaw, size,
    [max_acceleration, period](size_t i) { return max_acceleration * std::sin(2.0 * M_PI * i / period); });
}

Trajectory generateTrajectoryWithStepAcceleration(const double interval_distance, const double initial_speed,
  const double yaw, const size_t size, const std::vector<double> & values, const size_t steps_per_value) {
  return generateTrajectoryWithVariableAcceleration(interval_distance, initial_speed, yaw, size,
    [&values, steps_per_value](size_t i) { return values[(i / steps_per_value) % values.size()]; });
}

Trajectory generateTrajectory(const double interval_distance, const double speed, const double yaw, const size_t size) {
  return generateTrajectoryWithConstantAcceleration(interval_distance, speed, yaw, size, 0.0);
}

Trajectory generateTrajectoryWithConstantCurvature(const double interval_distance, const double speed,
  const double curvature, const size_t size, const double wheelbase) {
  if (std::abs(curvature) < 1.0e-5) return generateTrajectory(interval_distance, speed, 0.0, size);
  const double steering = std::atan(curvature * wheelbase), radius = 1.0 / curvature;
  Trajectory t = stampedTrajectory(); double x = 0.0, y = 0.0, yaw = 0.0;
  for (size_t i = 0; i <= size; ++i) {
    t.points.push_back(makePoint(x, y, yaw, speed, 0.0, steering));
    const double prev_yaw = yaw;
    yaw += curvature * interval_distance;
    x += radius * (std::sin(yaw) - std::sin(prev_yaw));
    y -= radius * (std::cos(yaw) - std::cos(prev_yaw));
  }
  return t;
}

Trajectory generateTrajectoryWithConstantSteering(const double interval_distance, const double speed,
  const double steering, const size_t size, const double wheelbase) {
  return generateTrajectoryWithConstantCurvature(interval_distance, speed, std::tan(steering) / wheelbase, size, wheelbase);
}

Trajectory generateTrajectoryWithConstantSteeringRate(const double interval_distance, const double speed,
  const double steering_rate, const size_t size, const double wheelbase) {
  Trajectory t = stampedTrajectory(); double x = 0.0, y = 0.0, yaw = 0.0, steering = 0.0;
  constexpr double max_steering = M_PI / 3.0;
  for (size_t i = 0; i <= size; ++i) {
    steering = std::clamp(steering, -max_steering, max_steering);
    t.points.push_back(makePoint(x, y, yaw, speed, 0.0, steering));
    yaw += std::tan(steering) / wheelbase * interval_distance;
    x += interval_distance * std::cos(yaw); y += interval_distance * std::sin(yaw);
    steering = std::abs(speed) > 0.01 ? steering + steering_rate * interval_distance / speed
                                      : (steering_rate > 0.0 ? max_steering : -max_steering);
  }
  return t;
}

Trajectory generateShiftedTrajectory(const Trajectory & t, const double lat_shift, const double lon_shift, const size_t size) {
  Trajectory shifted = stampedTrajectory();
  if (std::abs(lat_shift) <= std::numeric_limits<double>::epsilon() && std::abs(lon_shift) <= std::numeric_limits<double>::epsilon() && size >= t.points.size()) { shifted.points = t.points; return shifted; }
  const size_t count = std::min(size, t.points.size());
  shifted.points.assign(t.points.begin(), t.points.begin() + count);
  for (auto & p : shifted.points) {
    const double yaw = std::atan2(2.0 * (p.pose.orientation.w * p.pose.orientation.z), 1.0 - 2.0 * p.pose.orientation.z * p.pose.orientation.z);
    p.pose.position.x += lon_shift * std::cos(yaw) - lat_shift * std::sin(yaw);
    p.pose.position.y += lon_shift * std::sin(yaw) + lat_shift * std::cos(yaw);
  }
  return shifted;
}

Trajectory generateNanTrajectory() { auto t = generateTrajectory(1.0); t.points.front().pose.position.x = NAN; return t; }
Trajectory generateInfTrajectory() { auto t = generateTrajectory(1.0); t.points.front().pose.position.x = INFINITY; return t; }
Trajectory generateBadCurvatureTrajectory() {
  Trajectory t = stampedTrajectory(); double y = 1.5;
  for (double s = 0.0; s <= 10.0; s += 1.0) { t.points.push_back(makePoint(s, y, 0.0, 1.0)); y *= -1.0; }
  return t;
}
Odometry generateDefaultOdometry(const double x, const double y, const double vx) { Odometry o; o.pose.pose.position.x=x; o.pose.pose.position.y=y; o.twist.twist.linear.x=vx; return o; }
AccelWithCovarianceStamped generateDefaultAcceleration(const double ax) { AccelWithCovarianceStamped a; a.accel.accel.linear.x=ax; return a; }

void DefaultParameters::apply(const ros::NodeHandle & node) const {
  // ROS2 permits dots in parameter identifiers. ROS1 parameter keys are graph
  // names, so retain their hierarchy by mapping every ROS2 dot to a slash.
  const auto ros1_key = [](std::string key) { std::replace(key.begin(), key.end(), '.', '/'); return key; };
  for (const auto & p : doubles) node.setParam(ros1_key(p.first), p.second);
  for (const auto & p : integers) node.setParam(ros1_key(p.first), p.second);
  for (const auto & p : booleans) node.setParam(ros1_key(p.first), p.second);
}

DefaultParameters getDefaultParameters() {
  DefaultParameters p;
  p.integers = {{"default_handling_type", DEFAULT_HANDLING_TYPE},{"diag_error_count_threshold",0}};
  p.booleans = {{"publish_diag",true},{"display_on_terminal",true},{"enable_soft_stop_on_prev_traj",false},
    {"latency_checker.enable",true},{"latency_checker.is_critical",false},{"trajectory_checker.interval.enable",true},{"trajectory_checker.interval.is_critical",false},
    {"trajectory_checker.relative_angle.enable",true},{"trajectory_checker.relative_angle.is_critical",false},{"trajectory_checker.curvature.enable",true},{"trajectory_checker.curvature.is_critical",false},
    {"trajectory_checker.lateral_accel.enable",true},{"trajectory_checker.lateral_accel.is_critical",false},{"trajectory_checker.max_lon_accel.enable",true},{"trajectory_checker.max_lon_accel.is_critical",false},
    {"trajectory_checker.min_lon_accel.enable",true},{"trajectory_checker.min_lon_accel.is_critical",false},{"trajectory_checker.lateral_jerk.enable",true},{"trajectory_checker.lateral_jerk.is_critical",false},
    {"trajectory_checker.distance_deviation.enable",true},{"trajectory_checker.distance_deviation.is_critical",false},{"trajectory_checker.lon_distance_deviation.enable",true},{"trajectory_checker.lon_distance_deviation.is_critical",false},
    {"trajectory_checker.velocity_deviation.enable",true},{"trajectory_checker.velocity_deviation.is_critical",false},{"trajectory_checker.yaw_deviation.enable",true},{"trajectory_checker.yaw_deviation.is_critical",false},
    {"trajectory_checker.steering.enable",true},{"trajectory_checker.steering.is_critical",false},{"trajectory_checker.steering_rate.enable",true},{"trajectory_checker.steering_rate.is_critical",false},
    {"trajectory_checker.forward_trajectory_length.enable",true},{"trajectory_checker.forward_trajectory_length.is_critical",false},{"trajectory_checker.trajectory_shift.enable",true},{"trajectory_checker.trajectory_shift.is_critical",true}};
  p.doubles = {{"planning_hz",PLANNING_HZ},{"soft_stop_deceleration",-1.0},{"soft_stop_jerk_lim",0.3},{"th_traffic_light_timeout",0.5},
    {"latency_checker.threshold",THRESHOLD_LATENCY},{"trajectory_checker.interval.threshold",THRESHOLD_INTERVAL},{"trajectory_checker.relative_angle.threshold",THRESHOLD_RELATIVE_ANGLE},
    {"trajectory_checker.curvature.threshold",THRESHOLD_CURVATURE},{"trajectory_checker.lateral_accel.threshold",THRESHOLD_LATERAL_ACC},{"trajectory_checker.max_lon_accel.threshold",THRESHOLD_LONGITUDINAL_MAX_ACC},
    {"trajectory_checker.min_lon_accel.threshold",THRESHOLD_LONGITUDINAL_MIN_ACC},{"trajectory_checker.lateral_jerk.threshold",THRESHOLD_LATERAL_JERK},{"trajectory_checker.distance_deviation.threshold",THRESHOLD_DISTANCE_DEVIATION},
    {"trajectory_checker.lon_distance_deviation.threshold",THRESHOLD_LONGITUDINAL_DISTANCE_DEVIATION},{"trajectory_checker.velocity_deviation.threshold",THRESHOLD_VELOCITY_DEVIATION},{"trajectory_checker.yaw_deviation.threshold",THRESHOLD_YAW_DEVIATION},
    {"trajectory_checker.yaw_deviation.th_trajectory_yaw_shift",PARAMETER_TH_TRAJECTORY_YAW_SHIFT},{"trajectory_checker.steering.threshold",THRESHOLD_STEERING},{"trajectory_checker.steering_rate.threshold",THRESHOLD_STEERING_RATE},
    {"trajectory_checker.forward_trajectory_length.acceleration",PARAMETER_FORWARD_TRAJECTORY_LENGTH_ACCELERATION},{"trajectory_checker.forward_trajectory_length.margin",PARAMETER_FORWARD_TRAJECTORY_LENGTH_MARGIN},
    {"trajectory_checker.trajectory_shift.lat_shift_th",THRESHOLD_LATERAL_SHIFT},{"trajectory_checker.trajectory_shift.forward_shift_th",THRESHOLD_FORWARD_SHIFT},{"trajectory_checker.trajectory_shift.backward_shift_th",THRESHOLD_BACKWARD_SHIFT},
    {"wheel_radius",0.5},{"wheel_width",0.2},{"wheel_base",WHEELBASE},{"wheel_tread",2.0},{"front_overhang",1.0},{"rear_overhang",1.0},{"left_overhang",0.5},{"right_overhang",0.5},{"vehicle_height",1.5},{"max_steer_angle",0.7}};
  return p;
}
}  // namespace autoware::planning_validator::test_utils
