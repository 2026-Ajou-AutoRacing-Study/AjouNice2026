#include "trajectory_validator/purepursuit_hud_panel.hpp"

#include <algorithm>
#include <cmath>

#include <QHBoxLayout>
#include <QLabel>
#include <QSizePolicy>
#include <QDockWidget>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz/panel_dock_widget.h>
#include <rviz/visualization_frame.h>
#include <rviz/visualization_manager.h>

namespace trajectory_validator
{
namespace
{
QLabel * makeValueLabel(const QString & color)
{
  auto * label = new QLabel("--", nullptr);
  label->setAlignment(Qt::AlignCenter);
  label->setMinimumWidth(190);
  label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  label->setStyleSheet(
    "QLabel {"
    " color: " + color + ";"
    " font: 700 18px 'DejaVu Sans';"
    " padding: 6px 18px;"
    " background: rgba(20, 24, 30, 220);"
    " border: 1px solid " + color + ";"
    " border-radius: 4px;"
    "}");
  return label;
}
}  // namespace

PurePursuitHudPanel::PurePursuitHudPanel(QWidget * parent) : rviz::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 4, 8, 4);
  layout->setSpacing(6);

  auto * top_row = new QHBoxLayout();
  top_row->setSpacing(10);
  auto * bottom_row = new QHBoxLayout();
  bottom_row->setSpacing(10);

  lookahead_label_ = makeValueLabel("#38d9ff");
  steering_label_ = makeValueLabel("#ffe066");
  speed_label_ = makeValueLabel("#69db7c");
  lateral_error_label_ = makeValueLabel("#ff8787");
  heading_error_label_ = makeValueLabel("#b197fc");
  lateral_error_max_label_ = makeValueLabel("#ff6b6b");
  heading_error_max_label_ = makeValueLabel("#9775fa");
  top_row->addWidget(lookahead_label_);
  top_row->addWidget(steering_label_);
  top_row->addWidget(speed_label_);
  bottom_row->addWidget(lateral_error_label_);
  bottom_row->addWidget(lateral_error_max_label_);
  bottom_row->addWidget(heading_error_label_);
  bottom_row->addWidget(heading_error_max_label_);
  layout->addLayout(top_row);
  layout->addLayout(bottom_row);
  setMinimumHeight(96);

  connect(this, &PurePursuitHudPanel::lookaheadTextChanged, this,
          &PurePursuitHudPanel::setLookaheadText, Qt::QueuedConnection);
  connect(this, &PurePursuitHudPanel::steeringTextChanged, this,
          &PurePursuitHudPanel::setSteeringText, Qt::QueuedConnection);
  connect(this, &PurePursuitHudPanel::speedTextChanged, this,
          &PurePursuitHudPanel::setSpeedText, Qt::QueuedConnection);
  connect(this, &PurePursuitHudPanel::lateralErrorTextChanged, this,
          &PurePursuitHudPanel::setLateralErrorText, Qt::QueuedConnection);
  connect(this, &PurePursuitHudPanel::headingErrorTextChanged, this,
          &PurePursuitHudPanel::setHeadingErrorText, Qt::QueuedConnection);
  connect(this, &PurePursuitHudPanel::lateralErrorMaxTextChanged, this,
          &PurePursuitHudPanel::setLateralErrorMaxText, Qt::QueuedConnection);
  connect(this, &PurePursuitHudPanel::headingErrorMaxTextChanged, this,
          &PurePursuitHudPanel::setHeadingErrorMaxText, Qt::QueuedConnection);

  lookahead_subscriber_ = node_handle_.subscribe(
    "/purepursuit_autoware/debug/lookahead_distance", 1,
    &PurePursuitHudPanel::lookaheadCallback, this);
  steering_subscriber_ = node_handle_.subscribe(
    "/purepursuit_autoware/debug/limited_steering_angle", 1,
    &PurePursuitHudPanel::steeringCallback, this);
  speed_subscriber_ = node_handle_.subscribe(
    "/Ego_topic", 1, &PurePursuitHudPanel::speedCallback, this);
  error_subscriber_ = node_handle_.subscribe(
    "/trajectory_validator/debug", 1, &PurePursuitHudPanel::errorCallback, this);
}

void PurePursuitHudPanel::onInitialize()
{
  auto * dock = qobject_cast<rviz::PanelDockWidget *>(parentWidget());
  auto * frame = dynamic_cast<rviz::VisualizationFrame *>(vis_manager_->getWindowManager());
  if (dock == nullptr || frame == nullptr) {
    return;
  }

  frame->addDockWidget(Qt::TopDockWidgetArea, dock);
  dock->setAllowedAreas(Qt::TopDockWidgetArea);
  dock->setFeatures(QDockWidget::NoDockWidgetFeatures);
  dock->setTitleBarWidget(new QWidget(dock));
}

void PurePursuitHudPanel::lookaheadCallback(const std_msgs::Float64::ConstPtr & message)
{
  if (std::isfinite(message->data)) {
    Q_EMIT lookaheadTextChanged(QString("LD  %1 m").arg(message->data, 0, 'f', 2));
  }
}

void PurePursuitHudPanel::steeringCallback(const std_msgs::Float64::ConstPtr & message)
{
  if (std::isfinite(message->data)) {
    constexpr double kRadiansToDegrees = 180.0 / M_PI;
    Q_EMIT steeringTextChanged(
      QString("STEER  %1 deg").arg(message->data * kRadiansToDegrees, 0, 'f', 1, QChar(' ')));
  }
}

void PurePursuitHudPanel::speedCallback(
  const morai_msgs::EgoVehicleStatus::ConstPtr & message)
{
  const double speed_kph = std::hypot(
    std::hypot(message->velocity.x, message->velocity.y), message->velocity.z);
  if (std::isfinite(speed_kph)) {
    Q_EMIT speedTextChanged(QString("SPEED  %1 kph").arg(speed_kph, 0, 'f', 1));
  }
}

void PurePursuitHudPanel::errorCallback(
  const trajectory_validator::TrajectoryDebug::ConstPtr & message)
{
  if (message->run_id != current_run_id_) {
    current_run_id_ = message->run_id;
    lateral_error_max_m_ = 0.0;
    heading_error_max_rad_ = 0.0;
  }

  if (std::isfinite(message->lateral_error_m)) {
    lateral_error_max_m_ = std::max(
      lateral_error_max_m_, std::abs(message->lateral_error_m));
    Q_EMIT lateralErrorTextChanged(
      QString("LAT  %1 m").arg(message->lateral_error_m, 0, 'f', 3));
    Q_EMIT lateralErrorMaxTextChanged(
      QString("L_MAX  %1 m").arg(lateral_error_max_m_, 0, 'f', 3));
  }
  if (std::isfinite(message->heading_error_rad)) {
    constexpr double kRadiansToDegrees = 180.0 / M_PI;
    heading_error_max_rad_ = std::max(
      heading_error_max_rad_, std::abs(message->heading_error_rad));
    Q_EMIT headingErrorTextChanged(
      QString("HEADING  %1 deg").arg(
        message->heading_error_rad * kRadiansToDegrees, 0, 'f', 1));
    Q_EMIT headingErrorMaxTextChanged(
      QString("H_MAX  %1 deg").arg(
        heading_error_max_rad_ * kRadiansToDegrees, 0, 'f', 1));
  }
}

void PurePursuitHudPanel::setLookaheadText(const QString & text)
{
  lookahead_label_->setText(text);
}

void PurePursuitHudPanel::setSteeringText(const QString & text)
{
  steering_label_->setText(text);
}

void PurePursuitHudPanel::setSpeedText(const QString & text)
{
  speed_label_->setText(text);
}

void PurePursuitHudPanel::setLateralErrorText(const QString & text)
{
  lateral_error_label_->setText(text);
}

void PurePursuitHudPanel::setHeadingErrorText(const QString & text)
{
  heading_error_label_->setText(text);
}

void PurePursuitHudPanel::setLateralErrorMaxText(const QString & text)
{
  lateral_error_max_label_->setText(text);
}

void PurePursuitHudPanel::setHeadingErrorMaxText(const QString & text)
{
  heading_error_max_label_->setText(text);
}

}  // namespace trajectory_validator

PLUGINLIB_EXPORT_CLASS(trajectory_validator::PurePursuitHudPanel, rviz::Panel)
