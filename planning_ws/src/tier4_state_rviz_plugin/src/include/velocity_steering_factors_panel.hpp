//
//  Copyright 2023 TIER IV, Inc. All rights reserved.
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
//

#ifndef VELOCITY_STEERING_FACTORS_PANEL_HPP_
#define VELOCITY_STEERING_FACTORS_PANEL_HPP_

#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QLayout>
#include <QTableWidget>
#include <autoware_adapi_v1_msgs/PlanningBehavior.h>
#include <autoware_adapi_v1_msgs/SteeringFactorArray.h>
#include <autoware_adapi_v1_msgs/VelocityFactorArray.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rviz/panel.h>

#include <memory>

namespace rviz_plugins
{
class VelocitySteeringFactorsPanel : public rviz::Panel
{
  using PlanningBehavior = autoware_adapi_v1_msgs::PlanningBehavior;
  using VelocityFactorArray = autoware_adapi_v1_msgs::VelocityFactorArray;
  using VelocityFactor = autoware_adapi_v1_msgs::VelocityFactor;
  using SteeringFactorArray = autoware_adapi_v1_msgs::SteeringFactorArray;
  using SteeringFactor = autoware_adapi_v1_msgs::SteeringFactor;

  Q_OBJECT

public:
  explicit VelocitySteeringFactorsPanel(QWidget * parent = nullptr);
  void onInitialize() override;

protected:
  static constexpr double JERK_DEFAULT = 1.0;
  static constexpr double DECEL_LIMIT_DEFAULT = 1.0;

  static QColor colorFreakPink() { return QColor{255, 0, 108}; }

  // Layout
  QGroupBox * makeVelocityFactorsGroup();
  QGroupBox * makeSteeringFactorsGroup();

  ros::NodeHandle nh_;

  // Planning
  QTableWidget * velocity_factors_table_{nullptr};
  QTableWidget * steering_factors_table_{nullptr};
  QDoubleSpinBox * jerk_input_{nullptr};
  QDoubleSpinBox * decel_limit_input_{nullptr};

  nav_msgs::Odometry::ConstPtr kinematic_state_;
  geometry_msgs::AccelWithCovarianceStamped::ConstPtr acceleration_;

  ros::Subscriber sub_kinematic_state_;
  ros::Subscriber sub_acceleration_;
  ros::Subscriber sub_velocity_factors_;
  ros::Subscriber sub_steering_factors_;

  void onVelocityFactors(const VelocityFactorArray::ConstPtr & msg);
  void onSteeringFactors(const SteeringFactorArray::ConstPtr & msg);

  void save(rviz::Config config) const override;
  void load(const rviz::Config & config) override;
};
}  // namespace rviz_plugins

#endif  // VELOCITY_STEERING_FACTORS_PANEL_HPP_
