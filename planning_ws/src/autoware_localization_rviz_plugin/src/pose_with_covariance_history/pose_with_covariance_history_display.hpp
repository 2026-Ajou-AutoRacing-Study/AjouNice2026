// Copyright 2024 Tier IV, Inc.
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

#ifndef POSE_WITH_COVARIANCE_HISTORY__POSE_WITH_COVARIANCE_HISTORY_DISPLAY_HPP_
#define POSE_WITH_COVARIANCE_HISTORY__POSE_WITH_COVARIANCE_HISTORY_DISPLAY_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rviz/message_filter_display.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace rviz
{
class Shape;
class BillboardLine;
class Arrow;
}  // namespace rviz
namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class BoolProperty;
class EnumProperty;
}  // namespace rviz

namespace rviz_plugins
{
class PoseWithCovarianceHistory
: public rviz::MessageFilterDisplay<geometry_msgs::PoseWithCovarianceStamped>
{
  Q_OBJECT

public:
  PoseWithCovarianceHistory();
  ~PoseWithCovarianceHistory() override;
  PoseWithCovarianceHistory(const PoseWithCovarianceHistory &) = delete;
  PoseWithCovarianceHistory(const PoseWithCovarianceHistory &&) = delete;
  PoseWithCovarianceHistory & operator=(const PoseWithCovarianceHistory &) = delete;
  PoseWithCovarianceHistory & operator=(const PoseWithCovarianceHistory &&) = delete;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

private Q_SLOTS:
  void update_shape_type();

private:  // NOLINT : suppress redundancy warnings
          //          followings cannot be declared with the Q_SLOTS macro
  void subscribe() override;
  void unsubscribe() override;
  void processMessage(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & message) override;
  void update_history();
  void update_shapes();

  std::string target_frame_;
  std::deque<geometry_msgs::PoseWithCovarianceStamped::ConstPtr> history_;
  std::unique_ptr<rviz::BillboardLine> lines_;
  std::vector<std::unique_ptr<rviz::Shape>> spheres_;
  std::vector<std::unique_ptr<rviz::Arrow>> arrows_;
  ros::Time last_stamp_;

  rviz::BoolProperty * property_line_view_{};
  rviz::FloatProperty * property_line_width_;
  rviz::FloatProperty * property_line_alpha_;
  rviz::ColorProperty * property_line_color_;
  rviz::IntProperty * property_buffer_size_;

  rviz::BoolProperty * property_sphere_view_;
  rviz::FloatProperty * property_sphere_width_{};
  rviz::FloatProperty * property_sphere_alpha_;
  rviz::ColorProperty * property_sphere_color_;
  rviz::FloatProperty * property_sphere_scale_;

  rviz::BoolProperty * property_arrow_view_{};
  rviz::FloatProperty * property_arrow_shaft_length_;
  rviz::FloatProperty * property_arrow_shaft_diameter_;
  rviz::FloatProperty * property_arrow_head_length_;
  rviz::FloatProperty * property_arrow_head_diameter_;
  rviz::FloatProperty * property_arrow_alpha_;
  rviz::ColorProperty * property_arrow_color_;

  rviz::BoolProperty * property_path_view_;
  rviz::EnumProperty * property_shape_type_;
};

}  // namespace rviz_plugins

#endif  // POSE_WITH_COVARIANCE_HISTORY__POSE_WITH_COVARIANCE_HISTORY_DISPLAY_HPP_
