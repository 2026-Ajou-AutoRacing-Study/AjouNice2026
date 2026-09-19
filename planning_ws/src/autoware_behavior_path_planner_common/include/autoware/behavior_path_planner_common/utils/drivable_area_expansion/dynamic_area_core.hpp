#pragma once

#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/parameters.hpp>
#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/types.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <memory>
#include <vector>

namespace autoware::behavior_path_planner { struct PlannerData; }

namespace autoware::behavior_path_planner::drivable_area_expansion
{
void expand_drivable_area(PathWithLaneId & path, const std::shared_ptr<const autoware::behavior_path_planner::PlannerData> & planner_data);
void reuse_previous_poses(const PathWithLaneId & path, std::vector<geometry_msgs::Pose> & previous_poses, std::vector<double> & previous_curvatures, const geometry_msgs::Point & ego_point, const DrivableAreaExpansionParameters & params);
double calculate_minimum_lane_width(double curvature_radius, const DrivableAreaExpansionParameters & params);
Expansion calculate_expansion(const std::vector<geometry_msgs::Pose> & poses, const std::vector<geometry_msgs::Point> & left_bound, const std::vector<geometry_msgs::Point> & right_bound, const std::vector<double> & curvatures, const DrivableAreaExpansionParameters & params);
void apply_bound_change_rate_limit(std::vector<double> & distances, const std::vector<geometry_msgs::Point> & bound, double max_rate);
void calculate_expansion_distances(Expansion & expansion, const std::vector<double> & max_left_expansions, const std::vector<double> & max_right_expansions);
void add_bound_points(std::vector<geometry_msgs::Point> & left_bound, std::vector<geometry_msgs::Point> & right_bound, const std::vector<geometry_msgs::Pose> & poses, double minimum_interval);
std::vector<double> calculate_smoothed_curvatures(const std::vector<geometry_msgs::Pose> & poses, size_t smoothing_window_size);
void calculate_bound_index_mappings(Expansion & expansion, const std::vector<geometry_msgs::Pose> & poses, const std::vector<geometry_msgs::Point> & bound, Side side);
void apply_arc_length_range_smoothing(Expansion & expansion, const std::vector<geometry_msgs::Point> & bound, double arc_length_range, Side side);
void expand_bound(std::vector<geometry_msgs::Point> & bound, const std::vector<geometry_msgs::Pose> & poses, const std::vector<double> & expansions);
std::vector<double> calculate_maximum_distance_from_segments(const std::vector<geometry_msgs::Point> & bound, const SegmentRtree & segments, const DrivableAreaExpansionParameters & params, Side side);
std::vector<double> calculate_maximum_distance(const std::vector<geometry_msgs::Point> & bound, const SegmentRtree & segments, const std::vector<Polygon2d> & polygons, const DrivableAreaExpansionParameters & params, Side side);
}
