#ifndef AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__PARAMETERS_HPP_
#define AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__PARAMETERS_HPP_

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <ros/node_handle.h>

#include <optional>
#include <algorithm>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner::drivable_area_expansion
{
struct DrivableAreaExpansionParameters
{
  static constexpr auto DRIVABLE_AREA_RIGHT_BOUND_OFFSET_PARAM = "drivable_area_right_bound_offset";
  static constexpr auto DRIVABLE_AREA_LEFT_BOUND_OFFSET_PARAM = "drivable_area_left_bound_offset";
  static constexpr auto DRIVABLE_AREA_TYPES_TO_SKIP_PARAM = "drivable_area_types_to_skip";
  static constexpr auto ENABLED_PARAM = "dynamic_expansion.enabled";
  static constexpr auto EGO_EXTRA_FRONT_OVERHANG = "dynamic_expansion.ego.extra_front_overhang";
  static constexpr auto EGO_EXTRA_WHEELBASE = "dynamic_expansion.ego.extra_wheelbase";
  static constexpr auto EGO_EXTRA_WIDTH = "dynamic_expansion.ego.extra_width";
  static constexpr auto OBJECTS_SAFE_MARGIN_FRONT = "dynamic_expansion.object_exclusion.safety_margin.front";
  static constexpr auto OBJECTS_SAFE_MARGIN_REAR = "dynamic_expansion.object_exclusion.safety_margin.rear";
  static constexpr auto OBJECTS_SAFE_MARGIN_LEFT = "dynamic_expansion.object_exclusion.safety_margin.left";
  static constexpr auto OBJECTS_SAFE_MARGIN_RIGHT = "dynamic_expansion.object_exclusion.safety_margin.right";
  static constexpr auto STOPPED_OBJ_VEL_THRESH = "dynamic_expansion.object_exclusion.th_stopped_object_velocity";
  static constexpr auto MAX_EXP_DIST_PARAM = "dynamic_expansion.max_expansion_distance";
  static constexpr auto RESAMPLE_INTERVAL_PARAM = "dynamic_expansion.path_preprocessing.resample_interval";
  static constexpr auto MAX_PATH_ARC_LENGTH_PARAM = "dynamic_expansion.path_preprocessing.max_arc_length";
  static constexpr auto MAX_REUSE_DEVIATION_PARAM = "dynamic_expansion.path_preprocessing.reuse_max_deviation";
  static constexpr auto AVOID_STA_OBJECTS_PARAM = "dynamic_expansion.object_exclusion.exclude_static";
  static constexpr auto AVOID_DYN_OBJECTS_PARAM = "dynamic_expansion.object_exclusion.exclude_dynamic";
  static constexpr auto AVOID_LINESTRING_TYPES_PARAM = "dynamic_expansion.avoid_linestring.types";
  static constexpr auto AVOID_LINESTRING_DIST_PARAM = "dynamic_expansion.avoid_linestring.distance";
  static constexpr auto SMOOTHING_CURVATURE_WINDOW_PARAM = "dynamic_expansion.smoothing.curvature_average_window";
  static constexpr auto SMOOTHING_MAX_BOUND_RATE_PARAM = "dynamic_expansion.smoothing.max_bound_rate";
  static constexpr auto SMOOTHING_ARC_LENGTH_RANGE_PARAM = "dynamic_expansion.smoothing.arc_length_range";
  static constexpr auto PRINT_RUNTIME_PARAM = "dynamic_expansion.print_runtime";
  static constexpr auto MIN_BOUND_INTERVAL = "dynamic_expansion.min_bound_interval";

  double drivable_area_right_bound_offset{};
  double drivable_area_left_bound_offset{};
  std::vector<std::string> drivable_area_types_to_skip{};
  bool enabled{false};
  double avoid_linestring_dist{};
  double extra_front_overhang{};
  double extra_wheelbase{};
  double extra_width{};
  int curvature_average_window{};
  double max_bound_rate{};
  double max_expansion_distance{};
  double max_path_arc_length{};
  double resample_interval{};
  double arc_length_range{};
  double max_reuse_deviation{};
  double min_bound_interval{};
  bool print_runtime{};

  struct ObjectExclusion
  {
    bool exclude_static{};
    bool exclude_dynamic{};
    double front_offset{};
    double rear_offset{};
    double left_offset{};
    double right_offset{};
    double stopped_obj_vel_th{};
  } object_exclusion;

  struct LinestringType
  {
    std::string type;
    std::optional<std::string> subtype;
    explicit LinestringType(const std::string & s)
    {
      static const std::regex pattern(R"(^([^.]+)(?:\.(.*))?$)");
      std::smatch matches;
      if (std::regex_match(s, matches, pattern)) {
        type = matches[1].str();
        if (matches[2].matched) subtype = matches[2].str();
      }
    }
    [[nodiscard]] bool matches(const std::string & type_str, const std::string & subtype_str) const
    {
      return type == type_str && (!subtype.has_value() || *subtype == subtype_str);
    }
  };

  std::vector<LinestringType> avoid_linestring_types;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info;

  DrivableAreaExpansionParameters() = default;
  explicit DrivableAreaExpansionParameters(const ros::NodeHandle & node) { init(node); }

  void init(const ros::NodeHandle & node)
  {
    const auto required = [&node](const char * name, auto & value) {
      std::string key(name);
      std::replace(key.begin(), key.end(), '.', '/');
      if (!node.getParam(key, value)) {
        throw std::runtime_error("Missing required parameter `" + std::string(name) + "` in `" + node.getNamespace() + "`");
      }
    };
    required(DRIVABLE_AREA_RIGHT_BOUND_OFFSET_PARAM, drivable_area_right_bound_offset);
    required(DRIVABLE_AREA_LEFT_BOUND_OFFSET_PARAM, drivable_area_left_bound_offset);
    required(DRIVABLE_AREA_TYPES_TO_SKIP_PARAM, drivable_area_types_to_skip);
    required(ENABLED_PARAM, enabled); required(MAX_EXP_DIST_PARAM, max_expansion_distance);
    required(EGO_EXTRA_FRONT_OVERHANG, extra_front_overhang); required(EGO_EXTRA_WHEELBASE, extra_wheelbase);
    required(EGO_EXTRA_WIDTH, extra_width); required(SMOOTHING_CURVATURE_WINDOW_PARAM, curvature_average_window);
    required(SMOOTHING_MAX_BOUND_RATE_PARAM, max_bound_rate); required(SMOOTHING_ARC_LENGTH_RANGE_PARAM, arc_length_range);
    required(MAX_PATH_ARC_LENGTH_PARAM, max_path_arc_length); required(RESAMPLE_INTERVAL_PARAM, resample_interval);
    required(MAX_REUSE_DEVIATION_PARAM, max_reuse_deviation); required(AVOID_STA_OBJECTS_PARAM, object_exclusion.exclude_static);
    required(AVOID_DYN_OBJECTS_PARAM, object_exclusion.exclude_dynamic); required(STOPPED_OBJ_VEL_THRESH, object_exclusion.stopped_obj_vel_th);
    required(OBJECTS_SAFE_MARGIN_FRONT, object_exclusion.front_offset); required(OBJECTS_SAFE_MARGIN_REAR, object_exclusion.rear_offset);
    required(OBJECTS_SAFE_MARGIN_LEFT, object_exclusion.left_offset); required(OBJECTS_SAFE_MARGIN_RIGHT, object_exclusion.right_offset);
    std::vector<std::string> raw_linestring_types;
    required(AVOID_LINESTRING_TYPES_PARAM, raw_linestring_types);
    avoid_linestring_types.clear();
    for (const auto & raw_ls_type : raw_linestring_types) avoid_linestring_types.emplace_back(raw_ls_type);
    required(AVOID_LINESTRING_DIST_PARAM, avoid_linestring_dist); required(MIN_BOUND_INTERVAL, min_bound_interval);
    required(PRINT_RUNTIME_PARAM, print_runtime);
    vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  }
};
}  // namespace autoware::behavior_path_planner::drivable_area_expansion

#endif  // AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__PARAMETERS_HPP_
