#ifndef AUTOWARE_LC_SELECTED_OUTPUT_PREDICTION_HPP
#define AUTOWARE_LC_SELECTED_OUTPUT_PREDICTION_HPP
#include "autoware/behavior_path_lane_change_module/structs/data.hpp"
#include "autoware/behavior_path_lane_change_module/utils/frenet_safety.hpp"
namespace autoware::behavior_path_planner::utils::lane_change {
struct SelectedOutputPrediction {
  SafetyPrediction prediction;
  double prepare_time;
};
inline std::vector<SelectedOutputPrediction> selected_output_predictions(
  const SafetyPath & path,size_t begin,size_t end,size_t prepare_end,
  double speed,double acceleration,double maximum_speed,double resolution,
  double horizon,bool immediate,bool diagnostic_only)
{
  auto hypotheses=maneuver_motion_hypotheses(path,begin,end,speed,acceleration,
    maximum_speed,2.5,resolution,horizon);
  if(diagnostic_only)hypotheses.resize(1);
  std::vector<SelectedOutputPrediction> result;
  for(auto & hypothesis:hypotheses) {
    const auto remaining=remaining_prepare_time(path,prepare_end,hypothesis,immediate);
    result.push_back({std::move(hypothesis),remaining});
  }
  return result;
}
}
#endif
