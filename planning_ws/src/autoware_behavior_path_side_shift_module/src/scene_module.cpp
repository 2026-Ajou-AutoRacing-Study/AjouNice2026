#include <autoware/behavior_path_side_shift_module/scene_module.hpp>

#include <autoware/behavior_path_planner_common/utils/route_utils.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <cmath>

namespace autoware::behavior_path_planner
{
SideShiftSceneModule::SideShiftSceneModule(LegacySideShiftParameters parameters)
: SideShiftSceneModule(std::move(parameters), {}, {}, nullptr)
{
}

SideShiftSceneModule::SideShiftSceneModule(
  LegacySideShiftParameters parameters, RTCInterfaceMap rtc_interfaces,
  MarkerInterfaceMap marker_interfaces,
  std::shared_ptr<autoware::planning_factor_interface::PlanningFactorInterface> planning_factor_interface)
: SceneModuleInterface(
    "side_shift", std::move(rtc_interfaces), std::move(marker_interfaces),
    std::move(planning_factor_interface)),
  parameters_(std::move(parameters)), scene_(parameters_)
{
}

void SideShiftSceneModule::setLateralOffset(const double offset)
{
  requested_lateral_offset_ = offset;
  lateral_offset_change_request_ = true;
}

std::optional<ShiftLine> SideShiftSceneModule::latestShiftLine() const
{
  return latest_shift_line_;
}

void SideShiftSceneModule::updateData()
{
  if (!plannerData() || !plannerData()->lateral_offset) return;
  const auto & request = plannerData()->lateral_offset;
  // ROS1 rosidl-generated builtin_interfaces/Time is a plain sec/nanosec
  // message, whereas this candidate caches the source-clock value as
  // ros::Time.  Convert at the boundary before equality/subtraction.
  const ros::Time request_stamp(request->stamp.sec, request->stamp.nanosec);
  if (latest_lateral_offset_stamp_ && request_stamp == *latest_lateral_offset_stamp_) return;
  const auto elapsed = latest_lateral_offset_stamp_
    ? (request_stamp - *latest_lateral_offset_stamp_).toSec()
    : 0.0;
  if (latest_lateral_offset_stamp_ && parameters_.shift_request_time_limit > 0.0 &&
      elapsed < parameters_.shift_request_time_limit) {
    return;
  }
  requested_lateral_offset_ = request->lateral_offset;
  latest_lateral_offset_stamp_ = request_stamp;
  lateral_offset_change_request_ = true;
}

bool SideShiftSceneModule::isExecutionRequested() const
{
  return getCurrentStatus() == ModuleStatus::RUNNING ||
         (std::abs(requested_lateral_offset_) >= 1.0e-4 && plannerData() &&
          !getPreviousModuleOutput().reference_path.points.empty());
}

bool SideShiftSceneModule::canTransitSuccessState()
{
  return std::abs(requested_lateral_offset_) < 1.0e-4;
}

bool SideShiftSceneModule::isShifting(const size_t ego_index) const
{
  return latest_shift_line_ && ego_index >= latest_shift_line_->start_idx &&
         ego_index < latest_shift_line_->end_idx;
}

CandidateOutput SideShiftSceneModule::planCandidate() const
{
  return {};
}

BehaviorModuleOutput SideShiftSceneModule::plan()
{
  if (!plannerData() || getPreviousModuleOutput().reference_path.points.empty()) return {};
  const auto & reference_path = getPreviousModuleOutput().reference_path;
  const auto lanes = utils::getCurrentLanes(
    plannerData(), plannerData()->parameters.backward_path_length,
    plannerData()->parameters.forward_path_length);
  if (lanes.empty()) return {};
  const size_t ego_index = plannerData()->self_odometry
    ? autoware::motion_utils::findNearestIndex(
        reference_path.points, plannerData()->self_odometry->pose.pose.position)
    : 0U;

  // Upstream keeps a generated shifted path fixed while the ego vehicle is
  // inside its active shift interval.  A new lateral-offset request is held
  // until the interval is passed, preventing a mid-shift path replacement.
  if (isShifting(ego_index) && !previous_shifted_output_.path.points.empty()) {
    return previous_shifted_output_;
  }

  scene_.setReferencePath(reference_path);
  scene_.setCurrentLanelets(lanes);
  scene_.requestLateralOffset(requested_lateral_offset_);
  auto output = scene_.plan(
    ego_index, plannerData()->self_odometry ? plannerData()->self_odometry->twist.twist.linear.x : 0.0);
  if (output.path.points.empty()) return output;
  const auto lines = scene_.shiftLines();
  latest_shift_line_ = lines.empty() ? std::optional<ShiftLine>{} : std::optional<ShiftLine>{lines.front()};
  previous_shifted_output_ = output;
  lateral_offset_change_request_ = false;
  return output;
}
}  // namespace autoware::behavior_path_planner
