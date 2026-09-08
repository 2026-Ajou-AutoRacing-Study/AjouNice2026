//
//  Copyright 2025 TIER IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef AUTOWARE_PERCEPTION_RVIZ_PLUGIN__TRAFFIC_LIGHT__TRAFFIC_LIGHT_DISPLAY_HPP_
#define AUTOWARE_PERCEPTION_RVIZ_PLUGIN__TRAFFIC_LIGHT__TRAFFIC_LIGHT_DISPLAY_HPP_

#include <rviz/properties/ros_topic_property.h>
#include <rviz/ogre_helpers/shape.h>

#include <ros/ros.h>
#include <rviz/display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/ogre_helpers/movable_text.h>

#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_perception_msgs/TrafficLightGroupArray.h>

#include <boost/optional/optional.hpp>

#include <OgreSceneNode.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware_perception_rviz_plugin::traffic_light
{

struct Point3d
{
  double x;
  double y;
  double z;
};

struct TrafficLightBulbInfo
{
  lanelet::Id id;
  std::string color;
  std::string shape;
  Point3d position;
};

struct TrafficLightInfo
{
  lanelet::Id id;
  lanelet::ConstLineString3d linestring;
  boost::optional<double> height;
  std::vector<TrafficLightBulbInfo> bulbs;

  [[nodiscard]] TrafficLightBulbInfo getEstimatedBulb(const std::string & color) const;

  [[nodiscard]] Point3d getLinestringCenter() const;

  [[nodiscard]] std::vector<TrafficLightBulbInfo> getBlightBulbs(
    const std::vector<autoware_perception_msgs::TrafficLightElement> & current_elements) const;
};

class TrafficLightDisplay : public rviz::Display
{
  Q_OBJECT

public:
  TrafficLightDisplay();
  ~TrafficLightDisplay() override;

protected:
  void onInitialize() override;
  void reset() override;
  void update(float wall_dt, float ros_dt) override;
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void topic_updated_lanelet_map();
  void topic_updated_traffic_light();

private:  // NOLINT
  // Display methods
  void hideAllDisplays();
  void updateTrafficLightText(const TrafficLightInfo & info, const std::string & state_text);
  void updateTrafficLightBulbs(
    const TrafficLightInfo & info,
    const std::vector<autoware_perception_msgs::TrafficLightElement> & elements);
  bool checkTimeout() const;

  // Properties
  std::unique_ptr<rviz::RosTopicProperty> lanelet_map_topic_property_;
  std::unique_ptr<rviz::RosTopicProperty> traffic_light_topic_property_;
  std::unique_ptr<rviz::FloatProperty> timeout_property_;
  std::unique_ptr<rviz::FloatProperty> text_z_offset_property_;
  std::unique_ptr<rviz::FloatProperty> text_x_offset_property_;
  std::unique_ptr<rviz::FloatProperty> text_y_offset_property_;
  std::unique_ptr<rviz::BoolProperty> show_text_property_;
  std::unique_ptr<rviz::BoolProperty> show_bulb_property_;
  std::unique_ptr<rviz::StringProperty> text_prefix_property_;
  std::unique_ptr<rviz::FloatProperty> font_size_property_;
  std::unique_ptr<rviz::ColorProperty> text_color_property_;
  std::unique_ptr<rviz::FloatProperty> bulb_radius_property_;

  // Subscribers
  ros::Subscriber lanelet_map_sub_;
  ros::Subscriber traffic_light_group_array_sub_;

  // Callback functions
  void onLaneletMapReceived(const autoware_map_msgs::LaneletMapBin::ConstPtr msg);
  void onTrafficLightGroupArrayReceived(
    const autoware_perception_msgs::TrafficLightGroupArray::ConstPtr msg);

  std::mutex property_mutex_;
  std::mutex lanelet_map_mutex_;
  std::mutex traffic_light_mutex_;
  void setupRosSubscriptions();

  // Data storage
  lanelet::LaneletMapPtr lanelet_map_;
  std_msgs::Header lanelet_map_header_;

  autoware_perception_msgs::TrafficLightGroupArray::ConstPtr traffic_light_groups_;
  ros::Time last_traffic_light_received_time_;

  // Text visualization
  std::unordered_map<lanelet::Id, Ogre::SceneNode *> traffic_light_text_nodes_;
  std::unordered_map<lanelet::Id, std::unique_ptr<rviz::MovableText>>
    traffic_light_text_displays_;

  // Shape visualization
  std::unordered_map<lanelet::Id, std::unique_ptr<rviz::Shape>>
    traffic_light_bulb_displays_;
};

}  // namespace autoware_perception_rviz_plugin::traffic_light

#endif  // AUTOWARE_PERCEPTION_RVIZ_PLUGIN__TRAFFIC_LIGHT__TRAFFIC_LIGHT_DISPLAY_HPP_
