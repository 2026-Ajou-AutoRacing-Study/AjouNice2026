#ifndef TRAJECTORY_FOLLOWER_ADAPTIVE_PURE_PURSUIT_CORE_HPP_
#define TRAJECTORY_FOLLOWER_ADAPTIVE_PURE_PURSUIT_CORE_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace trajectory_follower
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct PathPoint
{
  Pose2D pose;
  double target_velocity_mps{0.0};
};

struct VehicleState
{
  Pose2D pose;
  double velocity_mps{0.0};
  double current_steering_tire_angle_rad{0.0};
};

struct AdaptiveLookaheadParams
{
  double wheel_base{3.0};
  double max_steering_angle{0.698};
  double ld_velocity_ratio{2.4};
  double ld_lateral_error_ratio{3.6};
  double ld_curvature_ratio{120.0};
  double long_ld_lateral_error_threshold{0.5};
  double min_lookahead_distance{4.35};
  double max_lookahead_distance{15.0};
  double reverse_min_lookahead_distance{7.0};
  double resampling_ds{0.1};
  double curvature_calculation_distance{4.0};
};

struct PurePursuitOutput
{
  bool valid{false};
  bool used_previous_command{false};
  std::string failure_reason;
  double steering_tire_angle_rad{0.0};
  double target_curvature{0.0};
  double lookahead_distance{0.0};
  double lateral_error{0.0};
  double path_curvature{0.0};
  double velocity_ld{0.0};
  double curvature_ld{0.0};
  double lateral_error_ld{0.0};
  Pose2D target_point;
};

// ROS-independent implementation of the Autoware Universe Pure Pursuit algorithm.
// The caller owns all message conversion and timeout handling.
class AdaptivePurePursuitCore
{
public:
  explicit AdaptivePurePursuitCore(const AdaptiveLookaheadParams & params);

  void setParams(const AdaptiveLookaheadParams & params);
  bool parametersValid(std::string * reason = nullptr) const;
  PurePursuitOutput run(const std::vector<PathPoint> & path, const VehicleState & state);

private:
  AdaptiveLookaheadParams params_;
  bool has_previous_command_{false};
  double previous_steering_command_{0.0};

  std::vector<PathPoint> resamplePath(const std::vector<PathPoint> & path) const;
  bool findNearestIndex(
    const std::vector<PathPoint> & path, const Pose2D & pose, std::size_t * index) const;
  double calculateLateralError(
    const std::vector<PathPoint> & path, const Pose2D & pose) const;
  double calculatePathCurvature(const std::vector<PathPoint> & path, std::size_t nearest_index) const;
  double calculateLookaheadDistance(
    double lateral_error, double path_curvature, double velocity, double min_lookahead,
    PurePursuitOutput * output) const;
  bool findLookaheadTarget(
    const std::vector<PathPoint> & path, std::size_t nearest_index, const Pose2D & pose,
    double lookahead_distance, Pose2D * target) const;
  static double calculatePurePursuitCurvature(const Pose2D & pose, const Pose2D & target);
  PurePursuitOutput failure(const std::string & reason) const;
};

}  // namespace trajectory_follower

#endif  // TRAJECTORY_FOLLOWER_ADAPTIVE_PURE_PURSUIT_CORE_HPP_
