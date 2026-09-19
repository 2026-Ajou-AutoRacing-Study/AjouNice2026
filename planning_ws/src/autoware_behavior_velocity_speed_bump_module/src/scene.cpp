#include "scene.hpp"
#include <autoware/trajectory/utils/crossed.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <algorithm>
#include <boost/geometry.hpp>
#include <cmath>

namespace autoware::behavior_velocity_planner::experimental
{
namespace bg = boost::geometry;
SpeedBumpModule::SpeedBumpModule(lanelet::Id id, const lanelet::autoware::SpeedBump & speed_bump,
  const PlannerParam & param, const ros::NodeHandle & node,
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
  std::shared_ptr<planning_factor_interface::PlanningFactorInterface> planning_factor_interface)
: SceneModuleInterface(id, node, std::move(time_keeper), std::move(planning_factor_interface)),
  id_(id), speed_bump_(speed_bump), param_(param)
{
  const auto height = static_cast<float>(speed_bump_.speedBump().attributeOr("height", 0.5));
  if (speed_bump_.speedBump().hasAttribute("slow_down_speed")) {
    slow_speed_ = static_cast<float>(speed_bump_.speedBump().attribute("slow_down_speed").asDouble().get() / 3.6);
  } else slow_speed_ = calculateSpeed(height, param_);
}
float SpeedBumpModule::calculateSpeed(float height, const PlannerParam & param)
{
  const auto width = param.max_height - param.min_height;
  if (std::abs(width) < 1e-6F) return std::min(param.min_speed, param.max_speed);
  const auto ratio = std::clamp((height - param.min_height) / width, 0.0F, 1.0F);
  return param.max_speed + ratio * (param.min_speed - param.max_speed);
}
bool SpeedBumpModule::modifyPathVelocity(Trajectory & path, const std::vector<geometry_msgs::Point> &,
  const std::vector<geometry_msgs::Point> &, const PlannerData & data)
{
  slow_start_s_.reset(); slow_end_s_.reset(); slow_start_pose_.reset();
  const auto polygon = lanelet::utils::to2D(speed_bump_.speedBump()).basicPolygon();
  auto crossings = autoware::experimental::trajectory::crossed_with_polygon(path, polygon);
  if (crossings.size() > 2U) crossings.resize(2U);
  const auto start_pose = path.compute(0.0).point.pose.position;
  const auto end_pose = path.compute(path.length()).point.pose.position;
  bg::model::polygon<bg::model::d2::point_xy<double>> boost_polygon;
  for (const auto & point : polygon) boost_polygon.outer().emplace_back(point.x(), point.y());
  if (!boost_polygon.outer().empty()) boost_polygon.outer().push_back(boost_polygon.outer().front());
  bg::correct(boost_polygon);
  const bg::model::d2::point_xy<double> first{start_pose.x, start_pose.y};
  const bg::model::d2::point_xy<double> last{end_pose.x, end_pose.y};
  const bool first_inside = bg::within(first, boost_polygon);
  const bool last_inside = bg::within(last, boost_polygon);
  if (crossings.empty() && !(first_inside && last_inside)) return false;
  double begin = 0.0, end = path.length();
  if (crossings.size() == 2U) { begin = crossings[0]; end = crossings[1]; }
  else if (crossings.size() == 1U) { if (last_inside) begin = crossings[0]; else if (first_inside) end = crossings[0]; else return false; }
  const double front = data.vehicle_info_.max_longitudinal_offset_m;
  const double rear = data.vehicle_info_.rear_overhang_m;
  begin = std::max(0.0, begin - front - param_.slow_start_margin);
  end = std::min(path.length(), end + rear + param_.slow_end_margin);
  if (begin > end) return false;
  path.longitudinal_velocity_mps().range(begin, end).clamp(slow_speed_);
  slow_start_s_ = begin; slow_end_s_ = end; slow_start_pose_ = path.compute(begin).point.pose;
  return true;
}
visualization_msgs::MarkerArray SpeedBumpModule::createDebugMarkerArray()
{
  visualization_msgs::MarkerArray result;
  if (!slow_start_pose_) return result;
  visualization_msgs::Marker marker; marker.header.frame_id = "map"; marker.header.stamp = ros::Time::now();
  marker.ns = "speed_bump"; marker.id = static_cast<int>(id_); marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD; marker.pose = *slow_start_pose_;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.4; marker.color.b = 1.0; marker.color.a = 1.0;
  result.markers.push_back(marker); return result;
}
motion_utils::VirtualWalls SpeedBumpModule::createVirtualWalls() { return {}; }
}  // namespace autoware::behavior_velocity_planner::experimental
