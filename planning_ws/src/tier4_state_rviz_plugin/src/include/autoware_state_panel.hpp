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

#ifndef AUTOWARE_STATE_PANEL_HPP_
#define AUTOWARE_STATE_PANEL_HPP_

#include "custom_button.hpp"
#include "custom_container.hpp"
#include "custom_icon_label.hpp"
#include "custom_label.hpp"
#include "custom_segmented_button.hpp"
#include "custom_segmented_button_item.hpp"
#include "custom_slider.hpp"
#include "custom_toggle_switch.hpp"
#include "material_colors.hpp"

#include <QChar>
#include <QColor>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QString>
#include <QVBoxLayout>
#include <autoware/adapi_specs/fail_safe.hpp>
#include <autoware/adapi_specs/localization.hpp>
#include <autoware/adapi_specs/motion.hpp>
#include <autoware/adapi_specs/operation_mode.hpp>
#include <autoware/adapi_specs/routing.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <autoware_adapi_v1_msgs/LocalizationInitializationState.h>
#include <autoware_adapi_v1_msgs/MotionState.h>
#include <autoware_adapi_v1_msgs/MrmState.h>
#include <autoware_adapi_v1_msgs/OperationModeState.h>
#include <autoware_adapi_v1_msgs/RouteState.h>
#include <autoware_internal_planning_msgs/VelocityLimit.h>
#include <autoware_vehicle_msgs/GearReport.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <tier4_external_api_msgs/Emergency.h>
#include <tier4_external_api_msgs/SetEmergency.h>

#include <qlabel.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

namespace rviz_plugins
{
class AutowareStatePanel : public rviz::Panel
{
  using OperationModeState = autoware_adapi_v1_msgs::OperationModeState;
  using RouteState = autoware_adapi_v1_msgs::RouteState;
  using LocalizationInitializationState = autoware_adapi_v1_msgs::LocalizationInitializationState;
  using MotionState = autoware_adapi_v1_msgs::MotionState;
  using MRMState = autoware_adapi_v1_msgs::MrmState;
  using DiagnosticArray = diagnostic_msgs::DiagnosticArray;
  using DiagnosticStatus = diagnostic_msgs::DiagnosticStatus;
  using ChangeToAutonomous = autoware::adapi_specs::operation_mode::ChangeToAutonomous;
  using ChangeToStop = autoware::adapi_specs::operation_mode::ChangeToStop;
  using ChangeToLocal = autoware::adapi_specs::operation_mode::ChangeToLocal;
  using ChangeToRemote = autoware::adapi_specs::operation_mode::ChangeToRemote;
  using EnableAutowareControl = autoware::adapi_specs::operation_mode::EnableAutowareControl;
  using DisableAutowareControl = autoware::adapi_specs::operation_mode::DisableAutowareControl;
  using ClearRoute = autoware::adapi_specs::routing::ClearRoute;
  using InitializeLocalization = autoware::adapi_specs::localization::Initialize;
  using AcceptStart = autoware::adapi_specs::motion::AcceptStart;
  using ListMrmDescription = autoware::adapi_specs::fail_safe::MrmDescription;
  struct SetEmergency {
    using Service = tier4_external_api_msgs::SetEmergency;
    static constexpr char name[] = "/api/autoware/set/emergency";
  };

  Q_OBJECT

public:
  explicit AutowareStatePanel(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:  // NOLINT for Qt
  void onClickAutonomous();
  void onClickStop();
  void onClickLocal();
  void onClickRemote();
  void onClickAutowareControl();
  void onClickDirectControl();
  void onClickClearRoute();
  void onClickInitByGnss();
  void onClickAcceptStart();
  void onClickVelocityLimit();
  void onClickEmergencyButton();
  void onSwitchStateChanged(int state);

protected:
  // Layout
  QVBoxLayout * makeVelocityLimitGroup();
  QVBoxLayout * makeOperationModeGroup();
  QVBoxLayout * makeRoutingGroup();
  QVBoxLayout * makeLocalizationGroup();
  QVBoxLayout * makeMotionGroup();
  QVBoxLayout * makeFailSafeGroup();
  // QVBoxLayout * makeDiagnosticGroup();

  // void onShift(const autoware_vehicle_msgs::msg::GearReport::ConstSharedPtr msg);
  void onEmergencyStatus(const tier4_external_api_msgs::Emergency::ConstPtr & msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_gear_;
  autoware::component_interface_utils::Client<SetEmergency>::SharedPtr client_emergency_stop_;
  ros::Subscriber sub_emergency_;
  ros::Publisher pub_velocity_limit_;

  QLabel * velocity_limit_value_label_{nullptr};
  bool sliderIsDragging = false;

  // Operation Mode
  QLabel * operation_mode_label_ptr_{nullptr};
  CustomSegmentedButtonItem * stop_button_ptr_{nullptr};
  CustomSegmentedButtonItem * auto_button_ptr_{nullptr};
  CustomSegmentedButtonItem * local_button_ptr_{nullptr};
  CustomSegmentedButtonItem * remote_button_ptr_{nullptr};

  ros::Subscriber sub_operation_mode_;
  autoware::component_interface_utils::Client<ChangeToAutonomous>::SharedPtr client_change_to_autonomous_;
  autoware::component_interface_utils::Client<ChangeToStop>::SharedPtr client_change_to_stop_;
  autoware::component_interface_utils::Client<ChangeToLocal>::SharedPtr client_change_to_local_;
  autoware::component_interface_utils::Client<ChangeToRemote>::SharedPtr client_change_to_remote_;

  //// Control Mode
  CustomSegmentedButton * segmented_button;
  CustomToggleSwitch * control_mode_switch_ptr_{nullptr};
  QLabel * control_mode_label_ptr_{nullptr};
  QPushButton * enable_button_ptr_{nullptr};
  QPushButton * disable_button_ptr_{nullptr};
  autoware::component_interface_utils::Client<EnableAutowareControl>::SharedPtr client_enable_autoware_control_;
  autoware::component_interface_utils::Client<DisableAutowareControl>::SharedPtr client_enable_direct_control_;

  //// Functions
  void onOperationMode(const OperationModeState::ConstPtr & msg);

  // Routing
  CustomIconLabel * routing_icon{nullptr};
  CustomElevatedButton * clear_route_button_ptr_{nullptr};
  QLabel * routing_label_ptr_{nullptr};

  ros::Subscriber sub_route_;
  autoware::component_interface_utils::Client<ClearRoute>::SharedPtr client_clear_route_;

  void onRoute(const RouteState::ConstPtr & msg);

  // Localization
  CustomIconLabel * localization_icon{nullptr};
  CustomElevatedButton * init_by_gnss_button_ptr_{nullptr};
  QLabel * localization_label_ptr_{nullptr};

  ros::Subscriber sub_localization_;
  autoware::component_interface_utils::Client<InitializeLocalization>::SharedPtr client_init_by_gnss_;

  void onLocalization(const LocalizationInitializationState::ConstPtr & msg);

  // Motion
  CustomIconLabel * motion_icon{nullptr};
  CustomElevatedButton * accept_start_button_ptr_{nullptr};
  QLabel * motion_label_ptr_{nullptr};

  ros::Subscriber sub_motion_;
  autoware::component_interface_utils::Client<AcceptStart>::SharedPtr client_accept_start_;

  void onMotion(const MotionState::ConstPtr & msg);

  // FailSafe
  CustomIconLabel * mrm_state_icon{nullptr};
  QLabel * mrm_state_label_ptr_{nullptr};
  CustomIconLabel * mrm_behavior_icon{nullptr};
  QLabel * mrm_behavior_label_ptr_{nullptr};
  std::unordered_map<uint16_t, std::pair<std::string, std::string>> mrm_behaviors_;

  ros::Subscriber sub_mrm_;
  autoware::component_interface_utils::Client<ListMrmDescription>::SharedPtr client_list_mrm_;
  ros::Timer timer_list_mrm_;

  void onMRMState(const MRMState::ConstPtr & msg);

  // Others
  QLabel * velocity_limit_setter_ptr_;
  QLabel * gear_label_ptr_;

  QSpinBox * pub_velocity_limit_input_;
  CustomElevatedButton * emergency_button_ptr_;

  bool current_emergency_{false};

  template <typename Spec>
  void callServiceWithoutResponse(
    const typename autoware::component_interface_utils::Client<Spec>::SharedPtr & client)
  {
    typename Spec::Service::Request req;
    ROS_DEBUG("state-panel client request: %s", Spec::name);

    if (!client->service_is_ready()) {
      ROS_DEBUG("state-panel client is unavailable: %s", Spec::name);
      return;
    }

    client->async_send_request(req, [](typename autoware::component_interface_utils::Client<Spec>::SharedFuture result) {
      const auto response = result.get();
      ROS_DEBUG("state-panel response: %d, %s", response->status.code, response->status.message.c_str());
    });
  }

  static void updateLabel(QLabel * label, QString text, QString style_sheet)
  {
    label->setText(text);
    label->setStyleSheet(style_sheet);
  }

  static void activateButton(QAbstractButton * button)
  {
    button->setChecked(false);
    button->setEnabled(true);
  }

  static void deactivateButton(QAbstractButton * button)
  {
    button->setChecked(true);
    button->setEnabled(false);
  }
};

}  // namespace rviz_plugins

#endif  // AUTOWARE_STATE_PANEL_HPP_
