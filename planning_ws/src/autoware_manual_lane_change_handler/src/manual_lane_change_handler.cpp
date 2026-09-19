#include "manual_lane_change_handler.hpp"

#include <algorithm>
#include <deque>
#include <iterator>

namespace autoware::manual_lane_change_handler
{
ManualLaneChangeHandler::ManualLaneChangeHandler() : pnh_("~")
{
  sub_map_ = pnh_.subscribe("input/vector_map", 1, &ManualLaneChangeHandler::map_callback, this);
  sub_odometry_ = pnh_.subscribe("input/odometry", 1, &ManualLaneChangeHandler::odometry_callback, this);
  sub_route_ = nh_.subscribe("/planning/mission_planning/route", 1, &ManualLaneChangeHandler::route_callback, this, ros::TransportHints().tcpNoDelay());
  sub_reroute_availability_ = pnh_.subscribe("input/reroute_availability", 1, &ManualLaneChangeHandler::reroute_callback, this);
  srv_set_preferred_lane_ = pnh_.advertiseService("set_preferred_lane", &ManualLaneChangeHandler::set_preferred_lane, this);
  client_set_preferred_primitive_ = nh_.serviceClient<autoware_planning_msgs::SetPreferredPrimitive>("/planning/mission_planning/mission_planner/set_preferred_primitive");
  pub_shift_number_ = pnh_.advertise<autoware_internal_debug_msgs::Int32Stamped>("debug/shift_number", 1);
}

void ManualLaneChangeHandler::map_callback(const autoware_map_msgs::LaneletMapBin::ConstPtr & msg)
{
  route_handler_.setMap(*msg);
}

std::vector<ManualLaneChangeHandler::Primitive> ManualLaneChangeHandler::sort_primitives_left_to_right(
  const Primitive & preferred, std::vector<Primitive> primitives) const
{
  std::deque<Primitive> sorted;
  const auto find_primitive = [&](lanelet::Id id) -> std::optional<Primitive> {
    const auto it = std::find_if(primitives.begin(), primitives.end(), [id](const Primitive & p) { return p.id == id; });
    return it == primitives.end() ? std::nullopt : std::optional<Primitive>(*it);
  };
  const auto current = route_handler_.getLaneletsFromId(preferred.id);
  for (auto left = route_handler_.getLeftLanelet(current, true); left; left = route_handler_.getLeftLanelet(*left, true)) if (const auto p = find_primitive(left->id())) sorted.push_front(*p);
  sorted.push_back(preferred);
  for (auto right = route_handler_.getRightLanelet(current, true); right; right = route_handler_.getRightLanelet(*right, true)) if (const auto p = find_primitive(right->id())) sorted.push_back(*p);
  return {sorted.begin(), sorted.end()};
}

void ManualLaneChangeHandler::route_callback(const autoware_planning_msgs::LaneletRoute::ConstPtr & msg)
{
  if (!route_handler_.isMapMsgReady()) { ROS_WARN("manual_lane_change_handler: route received before map"); return; }
  route_handler_.setRoute(*msg);
  auto route = *msg;
  for (auto & segment : route.segments) segment.primitives = sort_primitives_left_to_right(segment.preferred_primitive, segment.primitives);
  if (!current_route_ || msg->uuid != current_route_->uuid) { shift_number_ = 0; publish_shift_number(); }
  current_route_.reset(new autoware_planning_msgs::LaneletRoute(route));
  route_handler_.setRoute(*current_route_);
}

void ManualLaneChangeHandler::publish_shift_number()
{
  autoware_internal_debug_msgs::Int32Stamped msg;
  msg.stamp.sec = ros::Time::now().sec; msg.stamp.nanosec = ros::Time::now().nsec;
  msg.data = shift_number_;
  pub_shift_number_.publish(msg);
}

bool ManualLaneChangeHandler::set_preferred_lane(
  tier4_external_api_msgs::SetPreferredLane::Request & req,
  tier4_external_api_msgs::SetPreferredLane::Response & res)
{
  if (!current_route_ || !odometry_) { res.status.success = false; res.status.message = "No current route or odometry available."; return true; }
  if (!reroute_availability_ || !reroute_availability_->availability) { res.status.success = false; res.status.message = "Not in lane driving state."; return true; }
  if (!client_set_preferred_primitive_.waitForExistence(ros::Duration(5.0))) { res.status.success = false; res.status.message = "Mission planner preferred-primitive service unavailable."; return true; }
  lanelet::ConstLanelet closest = route_handler_.getLaneletMapPtr()->laneletLayer.get(current_route_->segments.front().preferred_primitive.id);
  if (!route_handler_.getClosestLaneletWithinRoute(odometry_->pose.pose, &closest)) { res.status.success = false; res.status.message = "Failed to find closest lanelet."; return true; }
  const auto decision = process_lane_change_request(closest.id(), req.lane_change_direction);
  if (!decision.success) { res.status.success = false; res.status.message = decision.message; return true; }
  autoware_planning_msgs::SetPreferredPrimitive request;
  request.request.preferred_primitives = decision.preferred_primitives;
  request.request.reset = req.lane_change_direction == tier4_external_api_msgs::SetPreferredLaneRequest::AUTO;
  request.request.uuid = current_route_->uuid;
  if (!client_set_preferred_primitive_.call(request)) { res.status.success = false; res.status.message = "Mission planner preferred-primitive call failed."; return true; }
  // Preserve the ROS2 asynchronous contract: this service acknowledges that a
  // valid override was accepted for forwarding.  The mission planner's own
  // response remains its separate authority and is not substituted here.
  res.status.success = decision.success;
  res.status.message = decision.message;
  return true;
}

LaneChangeRequestResult ManualLaneChangeHandler::process_lane_change_request(int64_t ego_lanelet_id, uint8_t requested)
{
  const auto direction = requested == tier4_external_api_msgs::SetPreferredLaneRequest::LEFT ? Direction::MANUAL_LEFT : requested == tier4_external_api_msgs::SetPreferredLaneRequest::RIGHT ? Direction::MANUAL_RIGHT : Direction::AUTO;
  if (direction == Direction::AUTO) { shift_number_ = 0; publish_shift_number(); return {{}, true, "Manual lane selection reset to AUTO."}; }
  if (!current_route_ || current_route_->segments.size() < 2) return {{}, false, "No route with at least two segments available."};
  auto route = *current_route_;
  auto begin = route.segments.begin(); const auto end = std::prev(route.segments.end());
  for (auto it = route.segments.begin(); it != route.segments.end(); ++it) {
    if (std::any_of(it->primitives.begin(), it->primitives.end(), [ego_lanelet_id](const Primitive & p) { return p.id == ego_lanelet_id; })) { begin = it; break; }
  }
  bool updated = false;
  for (auto it = begin; it != end; ++it) {
    auto & segment = *it; const auto next = std::next(it);
    const auto current_it = std::find_if(segment.primitives.begin(), segment.primitives.end(), [&](const Primitive & p) { return p.id == segment.preferred_primitive.id; });
    if (current_it == segment.primitives.end()) return {{}, false, "Current preferred primitive is absent from its segment."};
    const auto index = static_cast<size_t>(std::distance(segment.primitives.begin(), current_it));
    const auto next_it = std::find_if(next->primitives.begin(), next->primitives.end(), [&](const Primitive & p) { return p.id == next->preferred_primitive.id; });
    if (next_it == next->primitives.end()) return {{}, false, "Next preferred primitive is absent from its segment."};
    const auto turn = route_handler_.getLaneletMapPtr()->laneletLayer.get(next_it->id).attributeOr("turn_direction", "none");
    if ((direction == Direction::MANUAL_LEFT && index == 0) || (direction == Direction::MANUAL_RIGHT && index + 1 == segment.primitives.size()) || turn == "left" || turn == "right") break;
    segment.preferred_primitive = direction == Direction::MANUAL_LEFT ? segment.primitives.at(index - 1) : segment.primitives.at(index + 1);
    updated = true;
  }
  if (!updated) return {{}, false, "Manual lane selection is not possible for this route."};
  shift_number_ += direction == Direction::MANUAL_LEFT ? -1 : 1; publish_shift_number();
  std::vector<Primitive> preferred; preferred.reserve(route.segments.size());
  for (const auto & segment : route.segments) preferred.push_back(segment.preferred_primitive);
  return {preferred, true, direction == Direction::MANUAL_LEFT ? "Manual left lane selection executed." : "Manual right lane selection executed."};
}
}  // namespace autoware::manual_lane_change_handler

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "manual_lane_change_handler");
  autoware::manual_lane_change_handler::ManualLaneChangeHandler node;
  ros::spin();
  return 0;
}
