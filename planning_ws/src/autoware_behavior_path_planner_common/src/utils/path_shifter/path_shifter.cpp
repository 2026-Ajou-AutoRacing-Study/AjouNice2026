#include <autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp>

#include <autoware/behavior_path_planner_common/utils/path_utils.hpp>
#include <autoware/interpolation/spline_interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace autoware::behavior_path_planner
{
void PathShifter::setPath(const PathWithLaneId & path)
{
  reference_path_ = path;
  update_shift_lines_indices(shift_lines_);
  sort_shift_lines_along_path(shift_lines_);
}

void PathShifter::addShiftLine(const ShiftLine & line)
{
  shift_lines_.push_back(line);
  update_shift_lines_indices(shift_lines_);
  sort_shift_lines_along_path(shift_lines_);
}

void PathShifter::setShiftLines(const ShiftLineArray & lines)
{
  shift_lines_ = lines;
  update_shift_lines_indices(shift_lines_);
  sort_shift_lines_along_path(shift_lines_);
}

bool PathShifter::generate(ShiftedPath * shifted_path, const bool offset_back, const SHIFT_TYPE type) const
{
  if (!shifted_path || reference_path_.points.empty()) return false;
  shifted_path->path = reference_path_;
  shifted_path->shift_length.assign(reference_path_.points.size(), 0.0);
  if (shift_lines_.empty()) {
    shift_base_length(shifted_path, base_offset_);
    return true;
  }
  for (const auto & line : shift_lines_) {
    if (line.end_idx <= line.start_idx + 1) return false;
  }
  if (!check_shift_lines_alignment(shift_lines_)) return false;
  if (type == SHIFT_TYPE::SPLINE) apply_spline_shifter(shifted_path, offset_back);
  else apply_linear_shifter(shifted_path);
  shifted_path->path.points = autoware::motion_utils::removeOverlapPoints(shifted_path->path.points);
  autoware::motion_utils::removeFirstInvalidOrientationPoints(shifted_path->path.points);
  size_t previous_size{};
  do {
    previous_size = shifted_path->path.points.size();
    autoware::motion_utils::insertOrientation(shifted_path->path.points, true);
    autoware::motion_utils::removeFirstInvalidOrientationPoints(shifted_path->path.points);
  } while (previous_size != shifted_path->path.points.size());
  return true;
}

void PathShifter::apply_linear_shifter(ShiftedPath * shifted_path) const
{
  const auto arclength = utils::calcPathArcLengthArray(reference_path_);
  shift_base_length(shifted_path, base_offset_);
  for (const auto & line : shift_lines_) {
    const double current = shifted_path->shift_length.at(line.end_idx);
    const double delta = line.end_shift_length - current;
    const double length = std::max(arclength.at(line.end_idx) - arclength.at(line.start_idx), 1e-8);
    for (size_t index = 0; index < shifted_path->path.points.size(); ++index) {
      const double offset = index < line.start_idx ? 0.0 :
        index <= line.end_idx ? (arclength.at(index) - arclength.at(line.start_idx)) / length * delta : delta;
      add_lateral_offset_on_index_point(shifted_path, offset, index);
    }
  }
}

void PathShifter::apply_spline_shifter(ShiftedPath * shifted_path, const bool offset_back) const
{
  const auto arclength = utils::calcPathArcLengthArray(reference_path_);
  shift_base_length(shifted_path, base_offset_);
  for (const auto & line : shift_lines_) {
    const double delta = line.end_shift_length - shifted_path->shift_length.at(line.end_idx);
    const double length = std::max(arclength.at(line.end_idx) - arclength.at(line.start_idx), 1e-8);
    const auto bases = calc_base_lengths(length, delta, offset_back);
    std::vector<double> query;
    for (size_t index = line.start_idx + 1; index < line.end_idx; ++index)
      query.push_back(arclength.at(index) - arclength.at(line.start_idx));
    const auto offsets = query.empty() ? std::vector<double>{} : autoware::interpolation::spline(bases.first, bases.second, query);
    for (size_t index = 0; index < offsets.size(); ++index)
      add_lateral_offset_on_index_point(shifted_path, offsets.at(index), line.start_idx + 1 + index);
    if (offset_back) {
      for (size_t index = line.end_idx; index < shifted_path->path.points.size(); ++index)
        add_lateral_offset_on_index_point(shifted_path, delta, index);
    } else if (!offsets.empty()) {
      for (size_t index = 0; index <= line.start_idx; ++index)
        add_lateral_offset_on_index_point(shifted_path, offsets.front(), index);
    }
  }
}

std::pair<std::vector<double>, std::vector<double>> PathShifter::get_base_lengths_without_accel_limit(
  const double arclength, const double shift_length, const bool offset_back)
{
  std::vector<double> x{0.0, arclength * 0.25, arclength * 0.75, arclength};
  std::vector<double> y{0.0, shift_length / 12.0, shift_length * 11.0 / 12.0, shift_length};
  if (!offset_back) std::reverse(y.begin(), y.end());
  return {x, y};
}

std::pair<std::vector<double>, std::vector<double>> PathShifter::calc_base_lengths(
  const double arclength, const double shift_length, const bool offset_back) const
{
  const double velocity = std::abs(velocity_);
  const double acceleration = std::abs(longitudinal_acc_) > 1e-4 ? longitudinal_acc_ : 0.0;
  if (velocity < 1e-5 && acceleration <= 0.0) return get_base_lengths_without_accel_limit(arclength, shift_length, offset_back);
  const double final_velocity_squared = velocity * velocity + 2.0 * acceleration * arclength;
  if (final_velocity_squared < 0.0) {
    throw std::logic_error("Decelerating shift would stop before its end.");
  }
  const double time = acceleration != 0.0 ?
    2.0 * arclength / (velocity + std::sqrt(final_velocity_squared)) : arclength / velocity;
  const auto unconstrained = [&]() {
    auto lengths = get_base_lengths_without_accel_limit(arclength, shift_length, offset_back);
    // The lateral spline knots are time fractions, not distance fractions,
    // when braking. Preserve the historical nonnegative-acceleration path.
    if (acceleration < 0.0) {
      for (auto & s : lengths.first) {
        const double t = time * s / arclength;
        s = velocity * t + 0.5 * acceleration * t * t;
      }
    }
    return lengths;
  };
  const double max_lateral_acceleration = 8.0 * std::abs(shift_length) / (time * time);
  if (lateral_acc_limit_ <= 0.0 || max_lateral_acceleration < lateral_acc_limit_)
    return unconstrained();
  const double tj = time * 0.5 - 2.0 * std::abs(shift_length) / (lateral_acc_limit_ * time);
  const double ta = 4.0 * std::abs(shift_length) / (lateral_acc_limit_ * time) - time * 0.5;
  const double denominator = lateral_acc_limit_ * time * time - 4.0 * std::abs(shift_length);
  if (tj < 0.0 || ta < 0.0 || denominator <= 0.0 || tj / time < 0.1)
    return unconstrained();
  const double jerk = 2.0 * lateral_acc_limit_ * lateral_acc_limit_ * time / denominator;
  const double tj3 = tj * tj * tj, ta2tj = ta * ta * tj, tatj2 = ta * tj * tj;
  const double s1 = std::min(tj * velocity + 0.5 * acceleration * tj * tj, arclength);
  const double v1 = velocity + acceleration * tj;
  const double s2 = std::min(s1 + ta * v1 + 0.5 * acceleration * ta * ta, arclength);
  const double v2 = v1 + acceleration * ta;
  const double s3 = std::min(s2 + tj * v2 + 0.5 * acceleration * tj * tj, arclength);
  const double v3 = v2 + acceleration * tj;
  const double s5 = std::min(s3 + tj * v3 + 0.5 * acceleration * tj * tj, arclength);
  const double v5 = v3 + acceleration * tj;
  const double s6 = std::min(s5 + ta * v5 + 0.5 * acceleration * ta * ta, arclength);
  const double v6 = v5 + acceleration * ta;
  const double s7 = std::min(s6 + tj * v6 + 0.5 * acceleration * tj * tj, arclength);
  const double sign = shift_length >= 0.0 ? 1.0 : -1.0;
  std::vector<double> x{0.0, s1, s2, s3, s5, s6, s7};
  std::vector<double> y{0.0, sign * jerk * tj3 / 6.0,
    sign * (jerk * tj3 / 6.0 + 0.5 * jerk * tatj2 + 0.5 * jerk * ta2tj),
    sign * (jerk * tj3 + 1.5 * jerk * tatj2 + 0.5 * jerk * ta2tj),
    sign * (11.0 * jerk * tj3 / 6.0 + 2.5 * jerk * tatj2 + 0.5 * jerk * ta2tj),
    sign * (11.0 * jerk * tj3 / 6.0 + 3.0 * jerk * tatj2 + jerk * ta2tj),
    sign * (2.0 * jerk * tj3 + 3.0 * jerk * tatj2 + jerk * ta2tj)};
  if (!offset_back) std::reverse(y.begin(), y.end());
  return {x, y};
}

void PathShifter::update_shift_lines_indices(ShiftLineArray & lines) const
{
  if (reference_path_.points.empty()) return;
  for (auto & line : lines) {
    line.start_idx = autoware::motion_utils::findNearestIndex(reference_path_.points, line.start.position);
    line.end_idx = autoware::motion_utils::findNearestIndex(reference_path_.points, line.end.position);
  }
}

bool PathShifter::check_shift_lines_alignment(const ShiftLineArray & lines) const
{
  return std::all_of(lines.begin(), lines.end(), [](const auto & line) { return line.start_idx <= line.end_idx; });
}

void PathShifter::sort_shift_lines_along_path(ShiftLineArray & lines) const
{
  std::sort(lines.begin(), lines.end(), [](const auto & first, const auto & second) { return first.start_idx < second.start_idx; });
}

void PathShifter::removeBehindShiftLineAndSetBaseOffset(const size_t nearest_idx)
{
  ShiftLineArray retained, removed;
  for (const auto & line : shift_lines_) (line.end_idx > nearest_idx ? retained : removed).push_back(line);
  if (!removed.empty()) base_offset_ = std::max_element(removed.begin(), removed.end(), [](const auto & a, const auto & b) { return a.end_idx < b.end_idx; })->end_shift_length;
  if (std::abs(base_offset_) < 1e-4) base_offset_ = 0.0;
  setShiftLines(retained);
}

void PathShifter::add_lateral_offset_on_index_point(ShiftedPath * path, const double offset, const size_t index)
{
  if (std::abs(offset) < 1e-8) return;
  auto & pose = path->path.points.at(index).point.pose;
  const double yaw = tf2::getYaw(pose.orientation);
  pose.position.x -= std::sin(yaw) * offset;
  pose.position.y += std::cos(yaw) * offset;
  path->shift_length.at(index) += offset;
}

void PathShifter::shift_base_length(ShiftedPath * path, const double offset)
{
  if (std::abs(offset) <= 1e-4) return;
  for (size_t index = 0; index < path->path.points.size(); ++index) add_lateral_offset_on_index_point(path, offset, index);
}

double PathShifter::getLastShiftLength() const
{
  if (shift_lines_.empty()) return base_offset_;
  return std::max_element(shift_lines_.begin(), shift_lines_.end(), [](const auto & a, const auto & b) { return a.end_idx < b.end_idx; })->end_shift_length;
}

std::optional<ShiftLine> PathShifter::getLastShiftLine() const
{
  if (shift_lines_.empty()) return std::nullopt;
  return *std::max_element(shift_lines_.begin(), shift_lines_.end(), [](const auto & a, const auto & b) { return a.end_idx < b.end_idx; });
}
}  // namespace autoware::behavior_path_planner
