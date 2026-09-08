// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS__UTILS_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS__UTILS_HPP_


#include <cstddef>

namespace autoware::component_interface_specs
{

enum class QosReliability { Reliable };
enum class QosDurability { Volatile, TransientLocal };

// ROS1 creation policy. queue_size is passed to advertise/subscribe and
// latch is passed to advertise. Reliable records the ROS2 intent only: ROS1
// TCPROS has no DDS reliability negotiation or history policy.
struct Ros1Qos
{
  std::size_t queue_size;
  bool latch;
  QosReliability reliability;
  QosDurability durability;
};

template <typename T>
constexpr Ros1Qos get_qos()
{
  return {T::depth, T::durability == QosDurability::TransientLocal,
    T::reliability, T::durability};
}

}  // namespace autoware::component_interface_specs

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS__UTILS_HPP_
