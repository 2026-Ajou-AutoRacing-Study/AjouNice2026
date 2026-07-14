#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <XmlRpcValue.h>
#include <openssl/sha.h>
#include <tf2/utils.h>

#include "purepursuit_pi/controller.hpp"

namespace purepursuit_pi {
namespace {
double clamp(double value, double low, double high) { return std::max(low, std::min(high, value)); }

std::string sha256File(const std::string& filename) {
  std::ifstream stream(filename, std::ios::binary);
  if (!stream) return "unavailable";
  SHA256_CTX context;
  SHA256_Init(&context);
  char buffer[8192];
  while (stream.good()) {
    stream.read(buffer, sizeof(buffer));
    if (stream.gcount() > 0) SHA256_Update(&context, buffer, stream.gcount());
  }
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256_Final(digest, &context);
  std::ostringstream out;
  for (unsigned char byte : digest) out << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
  return out.str();
}

std::string fileText(const std::string& filename) {
  std::ifstream stream(filename);
  if (!stream) return "";
  std::ostringstream out; out << stream.rdbuf(); return out.str();
}
}  // namespace

Controller::Controller()
  : pnh_("~"), tf_listener_(tf_buffer_) {
  source_ = stringParam("ego_state/source", "morai_gt");
  config_path_ = stringParam("config_path", "");
  control_rate_hz_ = numberParam("controller/control_rate_hz", 20.0);
  trajectory_timeout_ = numberParam("controller/trajectory_timeout_sec", 2.0);
  ego_timeout_ = numberParam("controller/ego_timeout_sec", numberParam("ego_state/freshness_timeout_sec", 0.3));
  safe_stop_brake_ = numberParam("controller/safe_stop_brake", 0.2);
  vehicle_name_ = stringParam("vehicle/name", "2023_Hyundai_ioniq5");
  vehicle_.wheelbase_m = numberParam("vehicle/wheelbase_m", 3.0);
  lateral_.lookahead = numberParam("controller/lateral/fixed_lookahead_m", 5.0);
  lateral_.steer_limit = numberParam("controller/lateral/steering_limit_rad", 0.35);
  lateral_.steer_rate_limit = numberParam("controller/lateral/steering_rate_limit_radps", 0.7);
  longitudinal_.kp = numberParam("controller/longitudinal/kp", 1.0);
  longitudinal_.ki = numberParam("controller/longitudinal/ki", 0.3);
  longitudinal_.integral_limit = numberParam("controller/longitudinal/integral_limit", 4.0);
  longitudinal_.deadband = numberParam("controller/longitudinal/speed_deadband_mps", 0.05);
  longitudinal_.accel_gain = numberParam("controller/longitudinal/accel_gain", 0.12);
  longitudinal_.brake_gain = numberParam("controller/longitudinal/brake_gain", 0.16);
  longitudinal_.accel_limit = numberParam("controller/longitudinal/accel_limit", 0.55);
  longitudinal_.brake_limit = numberParam("controller/longitudinal/brake_limit", 0.65);
  longitudinal_.use_ff = boolParam("controller/longitudinal/use_acceleration_feedforward", false);
  if (source_ != "morai_gt" && source_ != "localization") {
    throw std::runtime_error("ego_state.source must be morai_gt or localization");
  }
  if (control_rate_hz_ < 20.0 || control_rate_hz_ > 50.0) {
    throw std::runtime_error("controller.control_rate_hz must be in [20, 50]");
  }
  trajectory_sub_ = nh_.subscribe("/planning/trajectory/reference", 1, &Controller::trajectoryCallback, this);
  if (source_ == "morai_gt") {
    ego_sub_ = nh_.subscribe(stringParam("ego_state/morai_gt_topic", "/Ego_topic"), 20, &Controller::moraiCallback, this);
  } else {
    odom_sub_ = nh_.subscribe(stringParam("ego_state/localization_odom_topic", "/localization/odom"), 20, &Controller::odomCallback, this);
  }
  command_pub_ = nh_.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 10);
  run_config_pub_ = nh_.advertise<std_msgs::String>("/control/purepursuit_pi/run_config", 1, true);
  ego_state_pub_ = nh_.advertise<nav_msgs::Odometry>("/control/purepursuit_pi/ego_state", 20);
  lookahead_pub_ = nh_.advertise<std_msgs::Float32>("/control/purepursuit_pi/debug/lookahead_distance", 10);
  elapsed_pub_ = nh_.advertise<std_msgs::Float32>("/control/purepursuit_pi/debug/control_tick_elapsed_ms", 10);
  deadline_pub_ = nh_.advertise<std_msgs::Bool>("/control/purepursuit_pi/debug/control_deadline_miss", 10);
  state_pub_ = nh_.advertise<std_msgs::String>("/control/purepursuit_pi/debug/state", 10, true);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/control/purepursuit_pi/debug/markers", 10);
  publishRunConfig();
  timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_hz_), &Controller::controlTick, this);
  ROS_INFO("purepursuit_pi ready: source=%s, %.1f Hz, wheelbase=%.3f, %s",
           source_.c_str(), control_rate_hz_, vehicle_.wheelbase_m, runLabel().c_str());
}

double Controller::numberParam(const std::string& key, double fallback) const {
  XmlRpc::XmlRpcValue value;
  if (!pnh_.getParam(key, value)) return fallback;
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(value);
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(value);
  ROS_WARN("purepursuit_pi parameter %s is not numeric; using %.3f", key.c_str(), fallback);
  return fallback;
}
bool Controller::boolParam(const std::string& key, bool fallback) const {
  bool value; return pnh_.param(key, value, fallback);
}
std::string Controller::stringParam(const std::string& key, const std::string& fallback) const {
  std::string value;
  pnh_.param(key, value, fallback);
  return value;
}

std::string Controller::runLabel() const {
  auto scaled = [](double value, double scale) { return static_cast<int>(std::lround(value * scale)); };
  std::ostringstream label;
  label << "pp_ld" << scaled(lateral_.lookahead, 10) << "_kp" << scaled(longitudinal_.kp, 100) << "_ki" << scaled(longitudinal_.ki, 100);
  return label.str();
}

void Controller::publishRunConfig() {
  std_msgs::String snapshot;
  std::ostringstream extra;
  extra << "\nrun_snapshot:\n"
        << "  controller: purepursuit_pi\n"
        << "  run_label: " << runLabel() << "\n"
        << "  config_path: " << config_path_ << "\n"
        << "  config_sha256: " << sha256File(config_path_) << "\n"
        << "  resolved_control_rate_hz: " << control_rate_hz_ << "\n"
        << "  ego_state_source: " << source_ << "\n"
        << "  normalized_ego_state_topic: /control/purepursuit_pi/ego_state\n"
        << "  map_frame: " << map_frame_ << "\n"
        << "  base_frame: " << base_frame_ << "\n"
        << "  vehicle_model: " << vehicle_name_ << "\n"
        << "  wheelbase_m: " << vehicle_.wheelbase_m << "\n"
        << "  steering_limit_rad: " << lateral_.steer_limit << "\n"
        << "  morai_longlCmdType: 1\n";
  snapshot.data = fileText(config_path_) + extra.str();
  run_config_pub_.publish(snapshot);
}

void Controller::trajectoryCallback(const autoware_planning_msgs::Trajectory::ConstPtr& message) {
  if (message->header.frame_id != map_frame_) {
    ROS_ERROR_THROTTLE(1.0, "purepursuit_pi expected reference frame %s, got %s", map_frame_.c_str(), message->header.frame_id.c_str());
    return;
  }
  std::vector<RefPoint> points;
  points.reserve(message->points.size());
  double s = 0.0;
  for (size_t i = 0; i < message->points.size(); ++i) {
    const auto& input = message->points[i];
    RefPoint point;
    point.x = input.pose.position.x; point.y = input.pose.position.y; point.z = input.pose.position.z;
    point.yaw = tf2::getYaw(input.pose.orientation); point.speed = input.longitudinal_velocity_mps; point.acceleration = input.acceleration_mps2;
    if (!points.empty()) s += std::hypot(point.x - points.back().x, point.y - points.back().y);
    point.s = s; points.push_back(point);
  }
  if (points.size() < 2) { ROS_ERROR_THROTTLE(1.0, "purepursuit_pi received trajectory with fewer than two points"); return; }
  std::lock_guard<std::mutex> lock(mutex_);
  trajectory_.set(std::move(points), message->header.stamp);
}

void Controller::controlTick(const ros::TimerEvent& event) {
  const ros::WallTime started = ros::WallTime::now();
  TrajectoryCache reference; EgoState ego;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reference = trajectory_; ego = ego_;
  }
  const ros::Time now = event.current_real;
  const double dt = last_tick_.isZero() ? 1.0 / control_rate_hz_ : std::max(1e-3, (now - last_tick_).toSec());
  last_tick_ = now;
  if (!reference.valid()) { publishSafeStop("WAIT_FOR_INPUT(reference)", now); return; }
  if (!ego.valid || (now - ego.stamp).toSec() > ego_timeout_) { publishSafeStop("STALE_EGO", now); return; }
  if (reference.stamp().isZero() || (now - reference.stamp()).toSec() > trajectory_timeout_) { publishSafeStop("STALE_REFERENCE", now); return; }
  const Projection projection = reference.project(ego.x, ego.y);
  if (!projection.valid) { publishSafeStop("PROJECTION_FAILURE", now); return; }
  if (projection.s >= reference.points().back().s - 0.05 && projection.point.speed <= 0.05) {
    publishSafeStop("FINISHED", now); return;
  }
  const RefPoint target = reference.atS(projection.s + lateral_.lookahead);
  const double raw_steer = computePurePursuit(ego, target, vehicle_, lateral_.lookahead);
  const double rate_limited = clamp(raw_steer, last_steer_ - lateral_.steer_rate_limit * dt, last_steer_ + lateral_.steer_rate_limit * dt);
  const double steer = clamp(rate_limited, -lateral_.steer_limit, lateral_.steer_limit);
  last_steer_ = steer;
  const double signed_output = computePI(projection.point.speed - ego.long_vel, projection.point.acceleration, dt, longitudinal_, &integral_);
  morai_msgs::CtrlCmd command;
  command.longlCmdType = 1; command.front_steer = steer; command.rear_steer = 0.0; command.velocity = 0.0; command.acceleration = 0.0;
  if (signed_output > longitudinal_.deadband) { command.accel = clamp(signed_output * longitudinal_.accel_gain, 0.0, longitudinal_.accel_limit); command.brake = 0.0; }
  else if (signed_output < -longitudinal_.deadband) { command.accel = 0.0; command.brake = clamp(-signed_output * longitudinal_.brake_gain, 0.0, longitudinal_.brake_limit); }
  else { command.accel = 0.0; command.brake = 0.0; }
  command_pub_.publish(command);
  std_msgs::Float32 lookahead; lookahead.data = lateral_.lookahead; lookahead_pub_.publish(lookahead);
  std_msgs::String state; state.data = "TRACKING"; state_pub_.publish(state); last_state_ = state.data;
  publishMarker(ego, projection, target, now);
  const double elapsed_ms = (ros::WallTime::now() - started).toSec() * 1000.0;
  std_msgs::Float32 elapsed; elapsed.data = elapsed_ms; elapsed_pub_.publish(elapsed);
  std_msgs::Bool deadline; deadline.data = elapsed_ms > 1000.0 / control_rate_hz_; deadline_pub_.publish(deadline);
}

void Controller::publishMarker(const EgoState& ego, const Projection& projection, const RefPoint& target, const ros::Time& stamp) {
  visualization_msgs::MarkerArray output;
  auto addSphere = [&](int id, const RefPoint& point, float r, float g, float b) {
    visualization_msgs::Marker marker; marker.header.frame_id = map_frame_; marker.header.stamp = stamp;
    marker.ns = "purepursuit"; marker.id = id; marker.type = visualization_msgs::Marker::SPHERE; marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point.x; marker.pose.position.y = point.y; marker.pose.position.z = point.z + 0.35; marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.55; marker.color.r = r; marker.color.g = g; marker.color.b = b; marker.color.a = 1.0; output.markers.push_back(marker);
  };
  addSphere(0, projection.point, 1.0f, 1.0f, 1.0f); addSphere(1, target, 1.0f, 0.2f, 1.0f);
  visualization_msgs::Marker error; error.header.frame_id = map_frame_; error.header.stamp = stamp;
  error.ns = "purepursuit"; error.id = 2; error.type = visualization_msgs::Marker::LINE_STRIP; error.action = visualization_msgs::Marker::ADD;
  error.scale.x = 0.08; error.color.r = 1.0; error.color.g = 0.0; error.color.b = 0.0; error.color.a = 1.0;
  geometry_msgs::Point a, b; a.x = ego.x; a.y = ego.y; a.z = ego.z + 0.2; b.x = projection.point.x; b.y = projection.point.y; b.z = projection.point.z + 0.2;
  error.points.push_back(a); error.points.push_back(b); output.markers.push_back(error);
  marker_pub_.publish(output);
}

void Controller::publishEgoState(const EgoState& ego) {
  nav_msgs::Odometry message;
  message.header.stamp = ego.stamp;
  message.header.frame_id = map_frame_;
  message.child_frame_id = base_frame_;
  message.pose.pose.position.x = ego.x;
  message.pose.pose.position.y = ego.y;
  message.pose.pose.position.z = ego.z;
  message.pose.pose.orientation.z = std::sin(ego.yaw * 0.5);
  message.pose.pose.orientation.w = std::cos(ego.yaw * 0.5);
  message.twist.twist.linear.x = ego.long_vel;
  message.twist.twist.linear.y = ego.lat_vel;
  ego_state_pub_.publish(message);
}
}  // namespace purepursuit_pi

int main(int argc, char** argv) {
  ros::init(argc, argv, "purepursuit_pi");
  try { purepursuit_pi::Controller controller; ros::spin(); }
  catch (const std::exception& error) { ROS_FATAL("purepursuit_pi startup failed: %s", error.what()); return 1; }
  return 0;
}
