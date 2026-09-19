#pragma once
#include <geometry_msgs/Pose.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <optional>
namespace autoware::path_generator::utils {
std::optional<lanelet::ConstPoint3d> get_border_point(const lanelet::BasicLineString3d &, const lanelet::ConstLanelet &);
bool is_pose_inside_lanelets(const geometry_msgs::Pose &, const lanelet::ConstLanelets &);
}
