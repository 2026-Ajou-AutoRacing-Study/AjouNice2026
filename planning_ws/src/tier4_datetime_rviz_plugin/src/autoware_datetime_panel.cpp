// Copyright 2021 Tier IV, Inc.
// Licensed under the Apache License, Version 2.0.
#include "autoware_datetime_panel.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QTimer>
#include <ros/time.h>

#include <ctime>

void set_format_time(QLineEdit * line, double time)
{
  char buffer[128];
  auto seconds = static_cast<time_t>(time);
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", localtime(&seconds));
  line->setText(QString(buffer) + QString::number((time - seconds), 'f', 3).rightRef(4));
}

AutowareDateTimePanel::AutowareDateTimePanel(QWidget * parent) : rviz::Panel(parent)
{
  ros_time_label_ = new QLineEdit; ros_time_label_->setReadOnly(true);
  wall_time_label_ = new QLineEdit; wall_time_label_->setReadOnly(true);
  auto * layout = new QHBoxLayout(this);
  layout->addWidget(new QLabel("ROS Time:")); layout->addWidget(ros_time_label_);
  layout->addWidget(new QLabel("Wall Time:")); layout->addWidget(wall_time_label_); setLayout(layout);
  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &AutowareDateTimePanel::update); timer->start(60);
}

void AutowareDateTimePanel::onInitialize() {}

void AutowareDateTimePanel::update()
{
  set_format_time(ros_time_label_, ros::Time::now().toSec());
  set_format_time(wall_time_label_, ros::WallTime::now().toSec());
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(AutowareDateTimePanel, rviz::Panel)
