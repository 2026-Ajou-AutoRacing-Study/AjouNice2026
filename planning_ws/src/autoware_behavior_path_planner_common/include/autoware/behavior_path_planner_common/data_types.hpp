#ifndef AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__DATA_TYPES_HPP_
#define AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__DATA_TYPES_HPP_

#include <autoware/behavior_path_planner_common/turn_signal_types.hpp>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_planning_msgs/PoseWithUuidStamped.h>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <geometry_msgs/Pose.h>
#include <unique_identifier_msgs/UUID.h>

#include <lanelet2_core/primitives/Lanelet.h>

#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
struct DrivableLanes
{
  lanelet::ConstLanelet right_lane;
  lanelet::ConstLanelet left_lane;
  lanelet::ConstLanelets middle_lanes;
};

struct DrivableAreaInfo
{
  struct Obstacle {
    geometry_msgs::Pose pose;
    autoware_utils_geometry::Polygon2d poly;
    bool is_left{true};
    bool direct_constraint{false};
    unique_identifier_msgs::UUID id;
    double hard_margin{0.0};
    double soft_margin{0.0};
  };
  std::vector<DrivableLanes> drivable_lanes;
  std::vector<Obstacle> obstacles;
  bool enable_expanding_hatched_road_markings{false};
  bool enable_expanding_intersection_areas{false};
  bool enable_expanding_freespace_areas{false};
  double drivable_margin{0.0};
  bool is_already_expanded{false};
};

struct BehaviorModuleOutput
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  autoware_internal_planning_msgs::PathWithLaneId reference_path;
  TurnSignalInfo turn_signal_info;
  std::optional<autoware_planning_msgs::PoseWithUuidStamped> modified_goal;
  DrivableAreaInfo drivable_area_info;
};

struct CandidateOutput
{
  CandidateOutput() = default;
  explicit CandidateOutput(autoware_internal_planning_msgs::PathWithLaneId path)
  : path_candidate(std::move(path)) {}
  autoware_internal_planning_msgs::PathWithLaneId path_candidate;
  double lateral_shift{0.0};
  double start_distance_to_path_change{std::numeric_limits<double>::lowest()};
  double finish_distance_to_path_change{std::numeric_limits<double>::lowest()};
};

struct PoseWithDetail
{
  geometry_msgs::Pose pose;
  std::string detail;
  explicit PoseWithDetail(const geometry_msgs::Pose & in_pose, std::string in_detail = "")
  : pose(in_pose), detail(std::move(in_detail)) {}
};
using PoseWithDetailOpt = std::optional<PoseWithDetail>;
}  // namespace autoware::behavior_path_planner

#endif
