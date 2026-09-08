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

#ifndef AUTOWARE_PLANNING_RVIZ_PLUGIN__CANDIDATE_TRAJECTORIES__DISPLAY_BASE_HPP_
#define AUTOWARE_PLANNING_RVIZ_PLUGIN__CANDIDATE_TRAJECTORIES__DISPLAY_BASE_HPP_

#include "autoware_planning_rviz_plugin/common/color_utils.hpp"

#include <ros/ros.h>
#include <rviz/display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/ogre_helpers/movable_text.h>

#include <autoware_internal_planning_msgs/CandidateTrajectories.h>
#include <autoware_internal_planning_msgs/ScoredCandidateTrajectories.h>

#include <OgreManualObject.h>
#include <OgreSceneNode.h>

#include <memory>
#include <vector>

namespace rviz_plugins
{

template <typename MessageType>
class CandidateTrajectoriesDisplayBase : public rviz::Display
{
public:
  CandidateTrajectoriesDisplayBase();
  virtual ~CandidateTrajectoriesDisplayBase();

  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void reset() override;

protected:
  // Pure virtual methods for message type specialization
  /// Process received ROS message and update visualization
  virtual void processMessage(const typename MessageType::ConstPtr msg_ptr) = 0;
  /// Setup coloring mode options specific to the message type
  virtual void setupColoringModes() = 0;

  // Common functionality
  virtual bool validateFloats(const typename MessageType::ConstPtr & msg_ptr);
  virtual std::unique_ptr<Ogre::ColourValue> setColorDependsOnVelocity(const double velocity);
  virtual std::unique_ptr<Ogre::ColourValue> gradation(
    const QColor & color_min, const QColor & color_max, const double ratio);

  // Common initialization methods
  void initializePropertyConstraints();

  // Common visualization methods
  void resizeManualObjects(size_t num_trajectories);
  void clearManualObjects();

  // Common properties - topic first for UI display order
  rviz::RosTopicProperty property_topic_;

  // Path visualization properties
  rviz::BoolProperty property_path_view_;
  rviz::BoolProperty property_path_width_view_;
  rviz::FloatProperty property_path_width_;
  rviz::FloatProperty property_path_alpha_;
  rviz::EnumProperty property_coloring_mode_;
  rviz::FloatProperty property_fade_out_distance_;

  // Velocity-based coloring properties (common to all)
  rviz::ColorProperty property_velocity_color_min_;
  rviz::ColorProperty property_velocity_color_mid_;
  rviz::ColorProperty property_velocity_color_max_;
  rviz::FloatProperty property_vel_max_;

  // Velocity visualization properties
  rviz::BoolProperty property_velocity_view_;
  rviz::FloatProperty property_velocity_alpha_;
  rviz::FloatProperty property_velocity_scale_;
  rviz::BoolProperty property_velocity_color_view_;
  rviz::ColorProperty property_velocity_color_;

  // Generator visualization properties
  rviz::BoolProperty property_generator_text_view_;
  rviz::FloatProperty property_generator_text_scale_;

  // Common data members
  std::vector<Ogre::ManualObject *> path_manual_objects_;
  std::vector<Ogre::ManualObject *> velocity_manual_objects_;
  std::vector<rviz::MovableText *> generator_texts_;
  std::vector<Ogre::SceneNode *> generator_text_nodes_;

  // Common coloring modes
  enum CommonColoringMode { VELOCITY_BASED = 0 };

  typename MessageType::ConstPtr last_msg_ptr_;

  // Common methods available to derived classes
  void subscribe();
  void unsubscribe();

private:
  ros::Subscriber subscription_;
};

}  // namespace rviz_plugins

#endif  // AUTOWARE_PLANNING_RVIZ_PLUGIN__CANDIDATE_TRAJECTORIES__DISPLAY_BASE_HPP_
