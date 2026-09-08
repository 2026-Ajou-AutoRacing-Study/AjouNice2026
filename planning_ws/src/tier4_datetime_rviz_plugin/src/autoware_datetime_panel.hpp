// Copyright 2021 Tier IV, Inc.
// Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE_DATETIME_PANEL_HPP_
#define AUTOWARE_DATETIME_PANEL_HPP_

#include <rviz/panel.h>

class QLineEdit;

class AutowareDateTimePanel : public rviz::Panel
{
  Q_OBJECT
public:
  explicit AutowareDateTimePanel(QWidget * parent = nullptr);
  void update();
  void onInitialize() override;
private:
  QLineEdit * ros_time_label_;
  QLineEdit * wall_time_label_;
};

#endif  // AUTOWARE_DATETIME_PANEL_HPP_
