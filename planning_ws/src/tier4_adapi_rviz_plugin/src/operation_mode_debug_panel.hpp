//
//  Copyright 2020 TIER IV, Inc. All rights reserved.
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

#ifndef OPERATION_MODE_DEBUG_PANEL_HPP_
#define OPERATION_MODE_DEBUG_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <autoware/adapi_specs/operation_mode.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <ros/ros.h>
#include <rviz/panel.h>

namespace tier4_adapi_rviz_plugins
{

class OperationModeDebugPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit OperationModeDebugPanel(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:  // NOLINT for Qt
  void onClickAutonomous();
  void onClickStop();
  void onClickLocal();
  void onClickRemote();
  void onClickAutowareControl();
  void onClickDirectControl();

protected:
  using OperationModeState = autoware_adapi_v1_msgs::OperationModeState;
  using ChangeToAutonomous = autoware::adapi_specs::operation_mode::ChangeToAutonomous;
  using ChangeToStop = autoware::adapi_specs::operation_mode::ChangeToStop;
  using ChangeToLocal = autoware::adapi_specs::operation_mode::ChangeToLocal;
  using ChangeToRemote = autoware::adapi_specs::operation_mode::ChangeToRemote;
  using EnableAutowareControl = autoware::adapi_specs::operation_mode::EnableAutowareControl;
  using DisableAutowareControl = autoware::adapi_specs::operation_mode::DisableAutowareControl;

  QLabel * operation_mode_label_ptr_{nullptr};
  QPushButton * stop_button_ptr_{nullptr};
  QPushButton * auto_button_ptr_{nullptr};
  QPushButton * local_button_ptr_{nullptr};
  QPushButton * remote_button_ptr_{nullptr};

  QLabel * control_mode_label_ptr_{nullptr};
  QPushButton * enable_button_ptr_{nullptr};
  QPushButton * disable_button_ptr_{nullptr};

  ros::NodeHandle nh_;
  ros::Subscriber sub_operation_mode_;
  autoware::component_interface_utils::Client<ChangeToAutonomous>::SharedPtr client_change_to_autonomous_;
  autoware::component_interface_utils::Client<ChangeToStop>::SharedPtr client_change_to_stop_;
  autoware::component_interface_utils::Client<ChangeToLocal>::SharedPtr client_change_to_local_;
  autoware::component_interface_utils::Client<ChangeToRemote>::SharedPtr client_change_to_remote_;
  autoware::component_interface_utils::Client<EnableAutowareControl>::SharedPtr client_enable_autoware_control_;
  autoware::component_interface_utils::Client<DisableAutowareControl>::SharedPtr client_enable_direct_control_;

  void onOperationMode(const OperationModeState::ConstPtr & msg);
  void onOperationModeUi(const OperationModeState & msg);
};

}  // namespace tier4_adapi_rviz_plugins

#endif  // OPERATION_MODE_DEBUG_PANEL_HPP_
