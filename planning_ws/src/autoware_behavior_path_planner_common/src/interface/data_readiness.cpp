#include <autoware/behavior_path_planner_common/interface/data_readiness.hpp>

namespace autoware::behavior_path_planner
{
namespace
{
void recordMandatory(
  const bool received, const char * name, BppInputReadinessReport * report)
{
  if (!received) {
    report->not_received.emplace_back(name);
    report->status = DataReadyStatus::NOT_RECEIVED;
  }
}

void recordCyclic(
  const std::optional<double> & received_at_seconds, const double now_seconds,
  const double timeout_seconds, const char * name, BppInputReadinessReport * report)
{
  if (!received_at_seconds) {
    report->not_received.emplace_back(name);
    report->status = DataReadyStatus::NOT_RECEIVED;
    return;
  }
  if (now_seconds - *received_at_seconds > timeout_seconds) {
    report->timed_out.emplace_back(name);
    if (report->status == DataReadyStatus::SUCCESS) {
      report->status = DataReadyStatus::TIMEOUT;
    }
  }
}
}  // namespace

BppInputReadinessReport evaluateBppInputReadiness(
  const BppInputState & state, const double now_seconds,
  const double cyclic_message_timeout_seconds, const bool enable_traffic_signal_timeout)
{
  BppInputReadinessReport report;
  report.status = DataReadyStatus::SUCCESS;

  recordMandatory(state.route_received, "route", &report);
  recordMandatory(state.vector_map_received, "vector_map", &report);
  recordMandatory(state.scenario_received, "scenario", &report);
  recordMandatory(state.acceleration_received, "acceleration", &report);
  recordMandatory(state.operation_mode_received, "operation_mode", &report);

  recordCyclic(
    state.perception_received_at_seconds, now_seconds, cyclic_message_timeout_seconds,
    "perception_objects", &report);
  recordCyclic(
    state.odometry_received_at_seconds, now_seconds, cyclic_message_timeout_seconds, "odometry",
    &report);
  recordCyclic(
    state.occupancy_grid_received_at_seconds, now_seconds, cyclic_message_timeout_seconds,
    "occupancy_grid_map", &report);
  if (enable_traffic_signal_timeout) {
    recordCyclic(
      state.traffic_signal_received_at_seconds, now_seconds, cyclic_message_timeout_seconds,
      "traffic_signal", &report);
  }
  return report;
}
}  // namespace autoware::behavior_path_planner
