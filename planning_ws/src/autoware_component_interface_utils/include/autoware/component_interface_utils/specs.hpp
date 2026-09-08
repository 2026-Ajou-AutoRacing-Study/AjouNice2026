#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__SPECS_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__SPECS_HPP_

#include <autoware/component_interface_specs/utils.hpp>

namespace autoware::component_interface_utils
{
template <class SpecT>
constexpr autoware::component_interface_specs::Ros1Qos get_qos()
{
  return autoware::component_interface_specs::get_qos<SpecT>();
}
}  // namespace autoware::component_interface_utils

#endif
