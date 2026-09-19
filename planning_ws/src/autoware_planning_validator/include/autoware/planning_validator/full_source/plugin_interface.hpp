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

#ifndef AUTOWARE__PLANNING_VALIDATOR__PLUGIN_INTERFACE_FULL_SOURCE_HPP_
#define AUTOWARE__PLANNING_VALIDATOR__PLUGIN_INTERFACE_FULL_SOURCE_HPP_

#include "autoware/planning_validator/full_source/types.hpp"
#include "autoware_planning_validator/PlanningValidatorStatus.h"

#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware/planning_validator/full_source/native_support.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_validator::full_source
{
using autoware_planning_validator::PlanningValidatorStatus;

class PluginInterface
{
public:
  virtual ~PluginInterface() = default;
  virtual void init(
    validator_native::Node & node, const std::string & name,
    const std::shared_ptr<PlanningValidatorContext> & context) = 0;
  virtual void validate() = 0;
  virtual void setup_diag() = 0;
  virtual std::string get_module_name() const = 0;
  std::string logger_ = std::string{};

protected:
  std::string module_name_;
  std::shared_ptr<PlanningValidatorContext> context_;
  std::shared_ptr<validator_native::Clock> clock_{};

  std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_;
};

}  // namespace autoware::planning_validator::full_source

#endif  // AUTOWARE__PLANNING_VALIDATOR__PLUGIN_INTERFACE_FULL_SOURCE_HPP_
