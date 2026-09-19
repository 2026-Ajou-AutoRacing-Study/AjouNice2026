// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "decision_result.hpp"

#include <string>

namespace autoware::behavior_velocity_planner
{
std::string formatDecisionResult(const RoundaboutDecisionResult & decision_result, const bool int_activated)
{
  const auto rtc = "RTC: roundabout activated_ = " + std::to_string(int_activated) + "\n";

  if (std::holds_alternative<RoundaboutInternalError>(decision_result)) {
    const auto & state = std::get<RoundaboutInternalError>(decision_result);
    return rtc + "InternalError because " + state.error;
  }
  if (std::holds_alternative<RoundaboutOverPassJudge>(decision_result)) {
    const auto & state = std::get<RoundaboutOverPassJudge>(decision_result);
    return rtc + "OverPassJudge:\nsafety_report:" + state.safety_report +
           "\nevasive_report:" + state.evasive_report;
  }
  if (std::holds_alternative<RoundaboutCollisionStop>(decision_result)) {
    return rtc + "CollisionStop:\n";
  }
  if (std::holds_alternative<RoundaboutSafe>(decision_result)) {
    return rtc + "Safe:\n";
  }
  return rtc + "";
}

}  // namespace autoware::behavior_velocity_planner
