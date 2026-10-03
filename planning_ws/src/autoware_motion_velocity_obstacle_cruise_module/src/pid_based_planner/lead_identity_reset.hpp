#pragma once
#include <optional>

namespace autoware::motion_velocity_planner
{
// A measurement filter belongs to one tracked obstacle, not the ego actuator.
// Reset callback must NOT discard the ego acceleration/jerk continuity state.
template<class Identity,class Reset>
bool reset_lead_history_if_changed(const bool enabled,const std::optional<Identity> & current,
  std::optional<Identity> & previous,Reset reset)
{
  if(!enabled){previous.reset();return false;}
  if(!current){previous.reset();return false;}
  if(previous&&*previous==*current)return false;
  reset();
  previous=current;
  return true;
}
}  // namespace autoware::motion_velocity_planner
