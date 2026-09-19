#ifndef AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_SHIFTER__PATH_SHIFTER_HPP_
#define AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_SHIFTER__PATH_SHIFTER_HPP_

#include <autoware_utils/ros/uuid_helper.hpp>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Pose.h>
#include <unique_identifier_msgs/UUID.h>

#include <optional>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
using PathWithLaneId = autoware_internal_planning_msgs::PathWithLaneId;
using PathPointWithLaneId = autoware_internal_planning_msgs::PathPointWithLaneId;
using ShiftLineArray = std::vector<struct ShiftLine>;

struct ShiftLine
{
  ShiftLine() : id(autoware_utils::generate_uuid()) {}
  geometry_msgs::Pose start{};
  geometry_msgs::Pose end{};
  double start_shift_length{};
  double end_shift_length{};
  size_t start_idx{};
  size_t end_idx{};
  unique_identifier_msgs::UUID id{};
};

struct ShiftedPath { PathWithLaneId path{}; std::vector<double> shift_length{}; };
enum class SHIFT_TYPE { LINEAR = 0, SPLINE = 1 };

class PathShifter
{
public:
  void setPath(const PathWithLaneId & path);
  void setVelocity(double velocity) { velocity_ = velocity; }
  void setLateralAccelerationLimit(double limit) { lateral_acc_limit_ = limit; }
  void setLongitudinalAcceleration(double acceleration) { longitudinal_acc_ = acceleration; }
  void addShiftLine(const ShiftLine & line);
  void setShiftLines(const ShiftLineArray & lines);
  ShiftLineArray getShiftLines() const { return shift_lines_; }
  double getBaseOffset() const { return base_offset_; }
  PathWithLaneId getReferencePath() const { return reference_path_; }
  bool generate(ShiftedPath * shifted_path, bool offset_back = true, SHIFT_TYPE type = SHIFT_TYPE::SPLINE) const;
  void removeBehindShiftLineAndSetBaseOffset(size_t nearest_idx);
  double getLastShiftLength() const;
  std::optional<ShiftLine> getLastShiftLine() const;

private:
  PathWithLaneId reference_path_{};
  ShiftLineArray shift_lines_{};
  double base_offset_{0.0};
  double velocity_{0.0};
  double lateral_acc_limit_{-1.0};
  double longitudinal_acc_{0.0};
  std::pair<std::vector<double>, std::vector<double>> calc_base_lengths(double arclength, double shift_length, bool offset_back) const;
  static std::pair<std::vector<double>, std::vector<double>> get_base_lengths_without_accel_limit(double arclength, double shift_length, bool offset_back);
  void update_shift_lines_indices(ShiftLineArray & lines) const;
  void sort_shift_lines_along_path(ShiftLineArray & lines) const;
  bool check_shift_lines_alignment(const ShiftLineArray & lines) const;
  void apply_linear_shifter(ShiftedPath * shifted_path) const;
  void apply_spline_shifter(ShiftedPath * shifted_path, bool offset_back) const;
  static void add_lateral_offset_on_index_point(ShiftedPath * path, double offset, size_t index);
  static void shift_base_length(ShiftedPath * path, double offset);
};
}  // namespace autoware::behavior_path_planner

#endif
