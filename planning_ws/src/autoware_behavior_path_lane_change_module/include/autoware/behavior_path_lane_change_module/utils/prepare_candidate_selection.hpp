#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace autoware::behavior_path_planner::lane_change {
// Ranking only: callers must first complete all original safety checks.
inline double prepare_speed_loss(double measured_speed, double duration, double endpoint_speed)
{
  if (!std::isfinite(measured_speed) || !std::isfinite(duration) ||
      !std::isfinite(endpoint_speed) || measured_speed < 0 || duration < 0 || endpoint_speed < 0)
    return std::numeric_limits<double>::infinity();
  if (duration <= 1e-6) return 0.;
  return std::max(0., measured_speed - endpoint_speed);
}
class SafePrepareCandidateSelection {
public:
  bool consider(size_t index, bool safe, double loss,
    double progress=-std::numeric_limits<double>::infinity(),
    double preparation=std::numeric_limits<double>::infinity()) {
    if (!safe) return false;
    // Retain a safe fallback even when ranking data is unavailable.
    if (!std::isfinite(loss) || loss < 0) loss=std::numeric_limits<double>::infinity();
    if(!std::isfinite(progress)||progress<0)progress=-std::numeric_limits<double>::infinity();
    if(!std::isfinite(preparation)||preparation<0)preparation=std::numeric_limits<double>::infinity();
    const bool same_loss=loss==loss_ || (std::isfinite(loss+loss_) && std::abs(loss-loss_)<=1e-3);
    const bool same_progress=progress==progress_ ||
      (std::isfinite(progress+progress_) && std::abs(progress-progress_)<=1e-3);
    if (!index_ || loss + 1e-3 < loss_ ||
        (same_loss && (progress>progress_+1e-3 ||
          (same_progress && preparation<preparation_-1e-3)))) {
      index_=index;loss_=loss;progress_=progress;preparation_=preparation;return true;
    }
    return false;
  }
  bool no_prepare_loss() const {return index_ && loss_ <= 1e-3;}
  bool has_value() const {return index_.has_value();}
  template<class T> bool append_selected(std::vector<T> & candidates) const {
    if (!index_ || *index_ >= candidates.size()) return false;
    // Preserve all existing diagnostic indices, including rejected candidates.
    if (*index_+1 != candidates.size()) {const T selected=candidates[*index_];candidates.push_back(selected);}
    return true;
  }
private:
  std::optional<size_t> index_;
  double loss_=std::numeric_limits<double>::infinity();
  double progress_=-std::numeric_limits<double>::infinity();
  double preparation_=std::numeric_limits<double>::infinity();
};
}
