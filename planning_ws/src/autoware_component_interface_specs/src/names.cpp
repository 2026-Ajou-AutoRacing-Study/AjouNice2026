#include <autoware/component_interface_specs/control.hpp>
#include <autoware/component_interface_specs/localization.hpp>
#include <autoware/component_interface_specs/map.hpp>
#include <autoware/component_interface_specs/perception.hpp>
#include <autoware/component_interface_specs/planning.hpp>
#include <autoware/component_interface_specs/system.hpp>
#include <autoware/component_interface_specs/vehicle.hpp>

namespace autoware::component_interface_specs
{
constexpr char control::ControlCommand::name[];
constexpr char localization::Initialize::name[];
constexpr char localization::InitializationState::name[];
constexpr char localization::KinematicState::name[];
constexpr char localization::Acceleration::name[];
constexpr char map::MapProjectorInfo::name[];
constexpr char map::PointCloudMap::name[];
constexpr char map::VectorMap::name[];
constexpr char perception::ObjectRecognition::name[];
constexpr char planning::SetLaneletRoute::name[];
constexpr char planning::SetWaypointRoute::name[];
constexpr char planning::ClearRoute::name[];
constexpr char planning::RouteState::name[];
constexpr char planning::LaneletRoute::name[];
constexpr char planning::Trajectory::name[];
constexpr char system::ChangeAutowareControl::name[];
constexpr char system::ChangeOperationMode::name[];
constexpr char system::OperationModeState::name[];
constexpr char vehicle::SteeringStatus::name[];
constexpr char vehicle::GearStatus::name[];
constexpr char vehicle::TurnIndicatorStatus::name[];
constexpr char vehicle::HazardLightStatus::name[];
}  // namespace autoware::component_interface_specs
