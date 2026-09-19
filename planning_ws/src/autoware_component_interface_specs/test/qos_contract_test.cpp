#include <gtest/gtest.h>

#include <autoware/component_interface_specs/control.hpp>
#include <autoware/component_interface_specs/localization.hpp>
#include <autoware/component_interface_specs/map.hpp>
#include <autoware/component_interface_specs/perception.hpp>
#include <autoware/component_interface_specs/planning.hpp>
#include <autoware/component_interface_specs/system.hpp>
#include <autoware/component_interface_specs/vehicle.hpp>

using autoware::component_interface_specs::get_qos;

template <class T> void expect_qos(const bool latch)
{
  constexpr auto q = get_qos<T>();
  EXPECT_EQ(q.queue_size, 1U);
  EXPECT_EQ(q.latch, latch);
  EXPECT_EQ(q.reliability, autoware::component_interface_specs::QosReliability::Reliable);
}

TEST(QosContract, ExactLatchSet)
{
  using namespace autoware::component_interface_specs;
  expect_qos<control::ControlCommand>(false);
  expect_qos<localization::InitializationState>(true); expect_qos<localization::KinematicState>(false); expect_qos<localization::Acceleration>(false);
  expect_qos<map::MapProjectorInfo>(true); expect_qos<map::PointCloudMap>(true); expect_qos<map::VectorMap>(true);
  expect_qos<perception::ObjectRecognition>(false);
  expect_qos<planning::RouteState>(true); expect_qos<planning::LaneletRoute>(true); expect_qos<planning::Trajectory>(false);
  expect_qos<system::OperationModeState>(true);
  expect_qos<vehicle::SteeringStatus>(false); expect_qos<vehicle::GearStatus>(false); expect_qos<vehicle::TurnIndicatorStatus>(false); expect_qos<vehicle::HazardLightStatus>(false);
}

TEST(QosContract, PublicNamesRemainExact)
{
  using namespace autoware::component_interface_specs;
  EXPECT_STREQ(control::ControlCommand::name, "/control/command/control_cmd");
  EXPECT_STREQ(localization::Initialize::name, "/localization/initialize");
  EXPECT_STREQ(localization::InitializationState::name, "/localization/initialization_state");
  EXPECT_STREQ(localization::KinematicState::name, "/localization/kinematic_state");
  EXPECT_STREQ(localization::Acceleration::name, "/localization/acceleration");
  EXPECT_STREQ(map::MapProjectorInfo::name, "/map/map_projector_info");
  EXPECT_STREQ(map::PointCloudMap::name, "/map/point_cloud_map");
  EXPECT_STREQ(map::VectorMap::name, "/map/vector_map");
  EXPECT_STREQ(perception::ObjectRecognition::name, "/perception/object_recognition/objects");
  EXPECT_STREQ(planning::SetLaneletRoute::name, "/planning/set_lanelet_route");
  EXPECT_STREQ(planning::SetWaypointRoute::name, "/planning/set_waypoint_route");
  EXPECT_STREQ(planning::ClearRoute::name, "/planning/clear_route");
  EXPECT_STREQ(planning::RouteState::name, "/planning/route_state");
  EXPECT_STREQ(planning::LaneletRoute::name, "/planning/route");
  EXPECT_STREQ(planning::Trajectory::name, "/planning/trajectory");
  EXPECT_STREQ(system::ChangeAutowareControl::name,
    "/system/operation_mode/change_autoware_control");
  EXPECT_STREQ(system::ChangeOperationMode::name,
    "/system/operation_mode/change_operation_mode");
  EXPECT_STREQ(system::OperationModeState::name, "/system/operation_mode/state");
  EXPECT_STREQ(vehicle::SteeringStatus::name, "/vehicle/status/steering_status");
  EXPECT_STREQ(vehicle::GearStatus::name, "/vehicle/status/gear_status");
  EXPECT_STREQ(vehicle::TurnIndicatorStatus::name, "/vehicle/status/turn_indicators_status");
  EXPECT_STREQ(vehicle::HazardLightStatus::name, "/vehicle/status/hazard_lights_status");
}
