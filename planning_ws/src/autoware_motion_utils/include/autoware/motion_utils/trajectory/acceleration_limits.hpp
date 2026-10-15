#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autoware::motion_utils
{
// Positive propulsion operating envelope, not a brake limit or pedal calibration.
// Monotone cubic Hermite interpolation (PCHIP); no extrapolation beyond the table.
class AccelerationLimits
{
public:
  using Map = std::vector<std::pair<double, double>>;
  AccelerationLimits() = default;
  explicit AccelerationLimits(Map map) : map_(std::move(map))
  {
    for (size_t i = 0; i < map_.size(); ++i) {
      const auto [v, a] = map_[i];
      if (!std::isfinite(v) || !std::isfinite(a) || v < 0 || a <= 0 ||
          (i && (v <= map_[i - 1].first || a > map_[i - 1].second))) {
        throw std::invalid_argument("Acceleration map requires increasing speeds and positive nonincreasing limits");
      }
    }
    slope_.assign(map_.size(), 0.0);
    if (map_.size() < 2) return;
    std::vector<double> h, d;
    for (size_t i = 1; i < map_.size(); ++i) {
      h.push_back(map_[i].first - map_[i - 1].first);
      d.push_back((map_[i].second - map_[i - 1].second) / h.back());
    }
    if (map_.size() == 2) { slope_[0] = slope_[1] = d[0]; return; }
    const auto endpoint = [](double h0, double h1, double d0, double d1) {
      const double m = ((2 * h0 + h1) * d0 - h0 * d1) / (h0 + h1);
      if (m * d0 <= 0) return 0.0;
      return d0 * d1 <= 0 && std::abs(m) > 3 * std::abs(d0) ? 3 * d0 : m;
    };
    slope_.front() = endpoint(h[0], h[1], d[0], d[1]);
    slope_.back() = endpoint(h.back(), h[h.size() - 2], d.back(), d[d.size() - 2]);
    for (size_t i = 1; i + 1 < map_.size(); ++i) {
      if (d[i - 1] * d[i] <= 0) continue;
      const double w1 = 2 * h[i] + h[i - 1], w2 = h[i] + 2 * h[i - 1];
      slope_[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i]);
    }
  }
  bool enabled() const { return !map_.empty(); }
  double at(double speed) const
  {
    if (map_.empty()) return std::numeric_limits<double>::infinity();
    if (!std::isfinite(speed)) throw std::invalid_argument("Nonfinite acceleration lookup speed");
    speed = std::abs(speed);
    if (speed <= map_.front().first) return map_.front().second;
    if (speed >= map_.back().first) return map_.back().second;
    const auto upper = std::upper_bound(map_.begin(), map_.end(), speed,
      [](double v, const auto & p) { return v < p.first; });
    const size_t i = static_cast<size_t>(upper - map_.begin() - 1);
    const double h = map_[i + 1].first - map_[i].first, t = (speed - map_[i].first) / h;
    return (2*t*t*t - 3*t*t + 1)*map_[i].second + (t*t*t - 2*t*t + t)*h*slope_[i] +
      (-2*t*t*t + 3*t*t)*map_[i + 1].second + (t*t*t - t*t)*h*slope_[i + 1];
  }
  double limit(double speed, double requested) const { return std::min(requested, at(speed)); }

  // Constant-acceleration segment is bounded at its faster end. Monotonicity
  // guarantees feasibility everywhere in the segment, including coarse samples.
  double reachableVelocity(double v0, double ds, double requested) const
  {
    if (v0 < 0 || ds < 0 || requested < 0 || !std::isfinite(v0 + ds + requested))
      throw std::invalid_argument("Invalid acceleration reachability input");
    double lo = v0, hi = std::sqrt(v0*v0 + 2*requested*ds);
    if (!enabled()) return hi;
    for (int i = 0; i < 40; ++i) {
      const double mid = (lo + hi) * 0.5;
      if (mid*mid - v0*v0 <= 2*limit(mid, requested)*ds) lo = mid; else hi = mid;
    }
    return lo;
  }
private:
  Map map_;
  std::vector<double> slope_;
};
}  // namespace autoware::motion_utils
