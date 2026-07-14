#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <ros/ros.h>

namespace purepursuit_pi {

struct RefPoint {
  double x{0.0}, y{0.0}, z{0.0}, yaw{0.0}, s{0.0};
  double speed{0.0}, acceleration{0.0};
};

struct Projection {
  bool valid{false};
  double s{0.0}, distance{0.0}, lateral_error{0.0};
  RefPoint point;
};

inline double wrapAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

class TrajectoryCache {
 public:
  void set(std::vector<RefPoint> points, const ros::Time& stamp) { points_ = std::move(points); stamp_ = stamp; }
  bool valid() const { return points_.size() >= 2; }
  const ros::Time& stamp() const { return stamp_; }
  const std::vector<RefPoint>& points() const { return points_; }

  Projection project(double x, double y) const {
    Projection best;
    best.distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < points_.size(); ++i) {
      const auto& a = points_[i]; const auto& b = points_[i + 1];
      const double dx = b.x - a.x, dy = b.y - a.y;
      const double norm2 = dx * dx + dy * dy;
      if (norm2 < 1e-10) continue;
      const double raw = ((x - a.x) * dx + (y - a.y) * dy) / norm2;
      const double t = std::max(0.0, std::min(1.0, raw));
      const double px = a.x + t * dx, py = a.y + t * dy;
      const double ex = x - px, ey = y - py;
      const double d = std::hypot(ex, ey);
      if (d < best.distance) {
        best.valid = true; best.distance = d; best.s = a.s + t * (b.s - a.s);
        best.point = interpolateSegment(a, b, t);
        best.lateral_error = -std::sin(best.point.yaw) * ex + std::cos(best.point.yaw) * ey;
      }
    }
    return best;
  }

  RefPoint atS(double s) const {
    if (s <= points_.front().s) return points_.front();
    if (s >= points_.back().s) return points_.back();
    auto it = std::lower_bound(points_.begin(), points_.end(), s,
      [](const RefPoint& point, double target) { return point.s < target; });
    const auto& b = *it; const auto& a = *(it - 1);
    const double t = (s - a.s) / std::max(b.s - a.s, 1e-9);
    return interpolateSegment(a, b, t);
  }

 private:
  static RefPoint interpolateSegment(const RefPoint& a, const RefPoint& b, double t) {
    RefPoint result;
    result.x = a.x + t * (b.x - a.x); result.y = a.y + t * (b.y - a.y); result.z = a.z + t * (b.z - a.z);
    result.s = a.s + t * (b.s - a.s); result.yaw = a.yaw + t * wrapAngle(b.yaw - a.yaw);
    result.speed = a.speed + t * (b.speed - a.speed); result.acceleration = a.acceleration + t * (b.acceleration - a.acceleration);
    return result;
  }
  std::vector<RefPoint> points_;
  ros::Time stamp_;
};

}  // namespace purepursuit_pi
