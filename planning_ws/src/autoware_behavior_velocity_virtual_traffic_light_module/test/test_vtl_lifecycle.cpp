#include "scene.hpp"

#include <autoware/route_handler/route_handler.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <boost/make_shared.hpp>

#include <algorithm>

namespace
{
using Module = autoware::behavior_velocity_planner::VirtualTrafficLightModule;

lanelet::LineString3d line(const lanelet::Id id, const double x)
{
  return lanelet::LineString3d(
    id, lanelet::Points3d{lanelet::Point3d(id * 10 + 1, x, -2.0, 0.0),
                           lanelet::Point3d(id * 10 + 2, x, 2.0, 0.0)});
}

autoware_internal_planning_msgs::PathWithLaneId path(const lanelet::Id lane_id)
{
  autoware_internal_planning_msgs::PathWithLaneId result;
  for (int x = 0; x <= 10; ++x) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = static_cast<double>(x);
    point.point.pose.orientation.w = 1.0;
    point.point.longitudinal_velocity_mps = 5.0;
    point.lane_ids.push_back(lane_id);
    result.points.push_back(point);
  }
  return result;
}

void setEgo(autoware::behavior_velocity_planner::PlannerData * planner_data, const double x)
{
  auto pose = std::make_shared<geometry_msgs::PoseStamped>();
  pose->pose.position.x = x;
  pose->pose.orientation.w = 1.0;
  planner_data->current_odometry = pose;
  geometry_msgs::TwistStamped stopped;
  stopped.header.stamp = ros::Time::now();
  stopped.twist.linear.x = 0.0;
  planner_data->velocity_buffer.clear();
  planner_data->velocity_buffer.push_back(stopped);
}

tier4_v2x_msgs::VirtualTrafficLightStateArrayConstPtr state(
  const std::string & type, const std::string & id, const bool finalized)
{
  auto states = boost::make_shared<tier4_v2x_msgs::VirtualTrafficLightStateArray>();
  tier4_v2x_msgs::VirtualTrafficLightState item;
  const auto stamp = ros::Time::now();
  item.stamp.sec = stamp.sec;
  item.stamp.nanosec = stamp.nsec;
  item.type = type;
  item.id = id;
  item.approval = true;
  item.is_finalized = finalized;
  states->states.push_back(item);
  return states;
}

bool hasStop(const autoware_internal_planning_msgs::PathWithLaneId & target)
{
  return std::any_of(target.points.begin(), target.points.end(), [](const auto & point) {
    return point.point.longitudinal_velocity_mps == 0.0;
  });
}

TEST(VirtualTrafficLightR028Lifecycle, EnforcesTypeAndIdAndCompletesLifecycle)
{
  constexpr lanelet::Id kLaneId = 100;
  constexpr lanelet::Id kInstrumentId = 200;
  lanelet::LineString3d left(
    101, lanelet::Points3d{lanelet::Point3d(1011, 0.0, 1.0, 0.0),
                            lanelet::Point3d(1012, 10.0, 1.0, 0.0)});
  lanelet::LineString3d right(
    102, lanelet::Points3d{lanelet::Point3d(1021, 0.0, -1.0, 0.0),
                            lanelet::Point3d(1022, 10.0, -1.0, 0.0)});
  lanelet::Lanelet lane{kLaneId, left, right};
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lane);

  auto instrument = line(kInstrumentId, 4.0);
  instrument.attributes()["type"] = "gate";
  lanelet::RuleParameterMap parameters;
  parameters[lanelet::RoleName::Refers].push_back(instrument);
  parameters[lanelet::RoleName::RefLine].push_back(line(201, 3.0));
  parameters["start_line"].push_back(line(202, 1.0));
  parameters["end_line"].push_back(line(203, 7.0));
  const auto reg = lanelet::RegulatoryElementFactory::create(
    lanelet::autoware::VirtualTrafficLight::RuleName, 300, parameters);
  const auto vtl = std::dynamic_pointer_cast<lanelet::autoware::VirtualTrafficLight>(reg);
  ASSERT_TRUE(vtl);

  ros::NodeHandle node;
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>();
  route_handler->setMap(map);
  route_handler->setRouteLanelets({lane});
  auto planner_data = std::make_shared<autoware::behavior_velocity_planner::PlannerData>();
  planner_data->route_handler_ = route_handler;
  auto planning_factor = std::make_shared<autoware::planning_factor_interface::PlanningFactorInterface>(
    &node, "vtl_r028_lifecycle");
  const Module::PlannerParam params{10.0, 2.0, 0.1, 0.0, 1.57, true};
  Module module{1, kLaneId, *vtl, lane, params, node, nullptr, planning_factor};
  module.setPlannerData(planner_data);

  setEgo(planner_data.get(), 0.0);
  auto working_path = path(kLaneId);
  ASSERT_TRUE(module.modifyPathVelocity(&working_path));
  ASSERT_TRUE(module.getInfrastructureCommand());
  EXPECT_EQ(module.getInfrastructureCommand()->state, static_cast<uint8_t>(Module::State::NONE));

  module.setCorrespondingVirtualTrafficLightState(state("wrong_type", "200", false));
  setEgo(planner_data.get(), 2.0);
  working_path = path(kLaneId);
  ASSERT_TRUE(module.modifyPathVelocity(&working_path));
  EXPECT_EQ(module.getInfrastructureCommand()->state, static_cast<uint8_t>(Module::State::REQUESTING));
  EXPECT_TRUE(hasStop(working_path));

  module.setCorrespondingVirtualTrafficLightState(state("gate", "200", false));
  setEgo(planner_data.get(), 4.0);
  working_path = path(kLaneId);
  ASSERT_TRUE(module.modifyPathVelocity(&working_path));
  EXPECT_EQ(module.getInfrastructureCommand()->state, static_cast<uint8_t>(Module::State::PASSING));
  EXPECT_TRUE(hasStop(working_path));

  setEgo(planner_data.get(), 6.5);
  working_path = path(kLaneId);
  ASSERT_TRUE(module.modifyPathVelocity(&working_path));
  EXPECT_EQ(module.getInfrastructureCommand()->state, static_cast<uint8_t>(Module::State::FINALIZING));

  module.setCorrespondingVirtualTrafficLightState(state("gate", "200", true));
  setEgo(planner_data.get(), 8.0);
  working_path = path(kLaneId);
  ASSERT_TRUE(module.modifyPathVelocity(&working_path));
  EXPECT_EQ(module.getInfrastructureCommand()->state, static_cast<uint8_t>(Module::State::FINALIZED));
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_vtl_r028_lifecycle");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
