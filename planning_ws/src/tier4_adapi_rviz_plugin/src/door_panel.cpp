//  Copyright 2023 The Autoware Contributors
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

#include "door_panel.hpp"

#include <QMetaObject>

#include <memory>

namespace tier4_adapi_rviz_plugins
{

DoorPanel::DoorPanel(QWidget * parent) : rviz::Panel(parent)
{
  status_ = new QLabel("Trying to get door layout.");
  layout_ = new QGridLayout();
  layout_->addWidget(status_, 0, 0, 1, 4);
  setLayout(layout_);
}

void DoorPanel::onInitialize()
{
  const auto on_layout = [this](const autoware::component_interface_utils::Client<DoorLayout>::SharedFuture future) {
    const auto res = future.get();
    if (!res->status.success) {
      const std::string message = "failed to get layout: " + res->status.message;
      QMetaObject::invokeMethod(this, [this, message] { status_->setText(QString::fromStdString(message)); }, Qt::QueuedConnection);
      return;
    }
    const auto layouts = res->doors;
    QMetaObject::invokeMethod(this, [this, layouts] {
    for (size_t index = 0; index < layouts.size(); ++index) {
      const auto & layout = layouts[index];
      DoorUI door;
      door.description = new QLabel(QString::fromStdString(layout.description));
      door.status = new QLabel("unknown");
      door.open = new QPushButton("open");
      door.close = new QPushButton("close");
      doors_.push_back(door);

      layout_->addWidget(door.description, index + 1, 0);
      layout_->addWidget(door.status, index + 1, 1);
      layout_->addWidget(door.open, index + 1, 2);
      layout_->addWidget(door.close, index + 1, 3);

      using Command = autoware_adapi_v1_msgs::DoorCommand;
      const auto on_open = [this, index] { on_button(index, Command::OPEN); };
      const auto on_close = [this, index] { on_button(index, Command::CLOSE); };
      connect(door.open, &QPushButton::clicked, on_open);
      connect(door.close, &QPushButton::clicked, on_close);
    }
    status_->hide();
    }, Qt::QueuedConnection);
  };

  const auto on_status = [this](const DoorStatus::Message::ConstPtr & msg) {
    const auto doors = msg->doors;
    QMetaObject::invokeMethod(this, [this, doors] {
    using Status = autoware_adapi_v1_msgs::DoorStatus;
    if (doors_.size() != doors.size()) {
      return;
    }
    for (size_t index = 0; index < doors_.size(); ++index) {
      const auto & label = doors_[index].status;
      switch (doors[index].status) {
        case Status::NOT_AVAILABLE:
          label->setText("not available");
          break;
        case Status::OPENED:
          label->setText("opened");
          break;
        case Status::CLOSED:
          label->setText("closed");
          break;
        case Status::OPENING:
          label->setText("opening");
          break;
        case Status::CLOSING:
          label->setText("closing");
          break;
        default:
          label->setText("unknown");
          break;
      }
    }
    }, Qt::QueuedConnection);
  };

  const auto adaptor = autoware::component_interface_utils::NodeAdaptor(nh_);
  adaptor.init_cli(cli_command_);
  adaptor.init_cli(cli_layout_);
  sub_status_ = nh_.subscribe<DoorStatus::Message>(DoorStatus::name, 1, on_status);

  const DoorLayout::Service::Request req;
  cli_layout_->async_send_request(req, on_layout);
}

void DoorPanel::on_button(uint32_t index, uint8_t command)
{
  DoorCommand::Service::Request req;
  req.doors.resize(1);
  req.doors.back().index = index;
  req.doors.back().command = command;
  cli_command_->async_send_request(req);
}

}  // namespace tier4_adapi_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tier4_adapi_rviz_plugins::DoorPanel, rviz::Panel)
