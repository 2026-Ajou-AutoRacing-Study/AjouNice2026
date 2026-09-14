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

#include "autoware/traffic_light_utils/traffic_light_utils.hpp"
#include "gtest/gtest.h"

namespace autoware::traffic_light_utils
{

TEST(setSignalUnknown, set_signal_element)
{
  tier4_perception_msgs::TrafficLight test_signal;
  tier4_perception_msgs::TrafficLightElement element;
  element.color = tier4_perception_msgs::TrafficLightElement::RED;
  element.shape = tier4_perception_msgs::TrafficLightElement::CROSS;
  test_signal.elements.push_back(element);
  EXPECT_EQ(test_signal.elements[0].color, tier4_perception_msgs::TrafficLightElement::RED);
  EXPECT_EQ(test_signal.elements[0].shape, tier4_perception_msgs::TrafficLightElement::CROSS);
  setSignalUnknown(test_signal, 1.23f);
  EXPECT_EQ(
    test_signal.elements[0].color, tier4_perception_msgs::TrafficLightElement::UNKNOWN);
  EXPECT_EQ(
    test_signal.elements[0].shape, tier4_perception_msgs::TrafficLightElement::UNKNOWN);
  EXPECT_FLOAT_EQ(test_signal.elements[0].confidence, (float)1.23);
}

TEST(getTrafficLightCenter, get_signal)
{
  lanelet::LineString3d lineString;
  lanelet::Point3d p0(0, 0, 0, 0);
  lanelet::Point3d p1(1, 1, 1, 1);
  lanelet::Point3d p2(2, 2, 2, 2);
  lanelet::Point3d p3(3, 3, 3, 3);
  lineString.push_back(p0);
  lineString.push_back(p1);
  lineString.push_back(p2);
  lineString.push_back(p3);

  lanelet::ConstLineString3d test_light(lineString);
  EXPECT_FLOAT_EQ(getTrafficLightCenter(test_light).x(), (float)1.5);
  EXPECT_FLOAT_EQ(getTrafficLightCenter(test_light).y(), (float)1.5);
  EXPECT_FLOAT_EQ(getTrafficLightCenter(test_light).z(), (float)1.5);
}

namespace
{
autoware_perception_msgs::TrafficLightElement element(const uint8_t color, const uint8_t shape)
{
  autoware_perception_msgs::TrafficLightElement value;
  value.color = color;
  value.shape = shape;
  value.status = autoware_perception_msgs::TrafficLightElement::SOLID_ON;
  return value;
}

lanelet::ConstLanelet laneWithTurnDirection(const std::string & direction)
{
  lanelet::LineString3d left(
    101, {lanelet::Point3d(102, 0.0, 0.0, 0.0), lanelet::Point3d(103, 1.0, 0.0, 0.0)});
  lanelet::LineString3d right(
    104, {lanelet::Point3d(105, 0.0, 1.0, 0.0), lanelet::Point3d(106, 1.0, 1.0, 0.0)});
  lanelet::Lanelet lane(107, left, right);
  lane.attributes()["turn_direction"] = direction;
  return lane;
}
}  // namespace

TEST(isTrafficSignalStop, route_maneuver_controls_green_permission)
{
  using Element = autoware_perception_msgs::TrafficLightElement;
  autoware_perception_msgs::TrafficLightGroup green_circle;
  green_circle.elements.push_back(element(Element::GREEN, Element::CIRCLE));

  autoware_perception_msgs::TrafficLightGroup protected_left;
  protected_left.elements.push_back(element(Element::RED, Element::CIRCLE));
  protected_left.elements.push_back(element(Element::GREEN, Element::LEFT_ARROW));

  EXPECT_FALSE(isTrafficSignalStop(laneWithTurnDirection("straight"), green_circle));
  EXPECT_TRUE(isTrafficSignalStop(laneWithTurnDirection("left"), green_circle));
  EXPECT_FALSE(isTrafficSignalStop(laneWithTurnDirection("left"), protected_left));
  EXPECT_TRUE(isTrafficSignalStop(laneWithTurnDirection("straight"), protected_left));
}

}  // namespace autoware::traffic_light_utils
