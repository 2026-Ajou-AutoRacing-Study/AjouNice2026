// Copyright 2021 Apex.AI, Inc.
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
#ifndef AUTOWARE_PERCEPTION_RVIZ_PLUGIN__COMMON__COLOR_ALPHA_PROPERTY_HPP_
#define AUTOWARE_PERCEPTION_RVIZ_PLUGIN__COMMON__COLOR_ALPHA_PROPERTY_HPP_

#include "autoware_perception_rviz_plugin/visibility_control.hpp"

#include <rviz/display.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>

#include <std_msgs/ColorRGBA.h>

#include <memory>

namespace autoware
{
namespace rviz_plugins
{
namespace common
{
/// \brief Class to define Color and Alpha values as plugin properties
class AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC ColorAlphaProperty
{
public:
  /// \brief Constructor
  /// \param color_default Default value for color property
  /// \param alpha_default Default value for alpha property
  /// \param parent_property Parent property for the color and alpha properties. Memory managed
  ///        by the caller
  ColorAlphaProperty(
    const QColor & color_default, const float alpha_default,
    rviz::Property * parent_property);

  /// \brief Convert color and alpha to ColorRGBA type
  /// \return color and alpha values as ColorRGBA type
  operator std_msgs::ColorRGBA() const;

private:
  rviz::ColorProperty m_color_property;
  rviz::FloatProperty m_alpha_property;
};

}  // namespace common
}  // namespace rviz_plugins
}  // namespace autoware

#endif  // AUTOWARE_PERCEPTION_RVIZ_PLUGIN__COMMON__COLOR_ALPHA_PROPERTY_HPP_
