// Copyright 2022 Tier IV, Inc.
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

#ifndef POSE_HISTORY_FOOTPRINT__DISPLAY_HPP_
#define POSE_HISTORY_FOOTPRINT__DISPLAY_HPP_

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rviz/message_filter_display.h>

#include <geometry_msgs/PoseStamped.h>

#include <deque>
#include <memory>
#include <string>

namespace rviz
{
class BillboardLine;
}  // namespace rviz
namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class BoolProperty;
}  // namespace rviz

namespace rviz_plugins
{

using autoware::vehicle_info_utils::VehicleInfo;

class PoseHistoryFootprint
: public rviz::MessageFilterDisplay<geometry_msgs::PoseStamped>
{
  Q_OBJECT

public:
  PoseHistoryFootprint();
  ~PoseHistoryFootprint() override;
  PoseHistoryFootprint(const PoseHistoryFootprint &) = delete;
  PoseHistoryFootprint(const PoseHistoryFootprint &&) = delete;
  PoseHistoryFootprint & operator=(const PoseHistoryFootprint &) = delete;
  PoseHistoryFootprint & operator=(const PoseHistoryFootprint &&) = delete;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update_footprint();

private Q_SLOTS:
  void update_vehicle_info();

private:  // NOLINT : suppress redundancy warnings
          //          followings cannot be declared with the Q_SLOTS macro
  void subscribe() override;
  void unsubscribe() override;
  void processMessage(const geometry_msgs::PoseStamped::ConstPtr & message) override;
  void update_history(const geometry_msgs::PoseStamped::ConstPtr & message);

  std::string target_frame_;
  std::deque<geometry_msgs::PoseStamped::ConstPtr> history_;
  ros::Time last_stamp_;

  // pose history
  rviz::IntProperty * property_buffer_size_;

  // trajectory footprint
  Ogre::ManualObject * trajectory_footprint_manual_object_{};
  rviz::BoolProperty * property_trajectory_footprint_view_;
  rviz::ColorProperty * property_trajectory_footprint_color_;
  rviz::FloatProperty * property_trajectory_footprint_alpha_;
  rviz::FloatProperty * property_vehicle_length_;
  rviz::FloatProperty * property_vehicle_width_;
  rviz::FloatProperty * property_rear_overhang_;
  rviz::FloatProperty * property_offset_;
  rviz::FloatProperty * property_interval_;

  struct VehicleFootprintInfo
  {
    VehicleFootprintInfo(const float l, const float w, const float r)
    : length(l), width(w), rear_overhang(r)
    {
    }
    float length, width, rear_overhang;
  };
  std::shared_ptr<VehicleInfo> vehicle_info_;
  std::shared_ptr<VehicleFootprintInfo> vehicle_footprint_info_;

  geometry_msgs::PoseStamped::ConstPtr last_msg_ptr_;
};

}  // namespace rviz_plugins

#endif  // POSE_HISTORY_FOOTPRINT__DISPLAY_HPP_
