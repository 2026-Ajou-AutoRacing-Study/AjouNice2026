#include "trajectory_validator/planner_csv_logger_panel.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSizePolicy>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace trajectory_validator
{

PlannerCsvLoggerPanel::PlannerCsvLoggerPanel(QWidget * parent) : rviz::Panel(parent)
{
  start_client_ = node_handle_.serviceClient<std_srvs::Trigger>(
    "/planner_csv_logger/start");
  stop_client_ = node_handle_.serviceClient<std_srvs::Trigger>(
    "/planner_csv_logger/stop");

  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(8);

  auto * title = new QLabel("Planner CSV Logger", this);
  title->setStyleSheet("QLabel { font: 700 15px 'DejaVu Sans'; }");
  layout->addWidget(title);

  status_label_ = new QLabel("Ready - logging stopped", this);
  status_label_->setWordWrap(true);
  status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  status_label_->setMinimumHeight(42);
  layout->addWidget(status_label_);

  auto * button_row = new QHBoxLayout();
  button_row->setSpacing(8);
  start_button_ = new QPushButton("Start Logging", this);
  stop_button_ = new QPushButton("Stop Logging", this);
  start_button_->setMinimumHeight(42);
  stop_button_->setMinimumHeight(42);
  start_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  stop_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  start_button_->setStyleSheet(
    "QPushButton { background: #2b8a3e; color: white; font-weight: 700; "
    "border-radius: 4px; padding: 8px; }"
    "QPushButton:pressed { background: #237032; }");
  stop_button_->setStyleSheet(
    "QPushButton { background: #c92a2a; color: white; font-weight: 700; "
    "border-radius: 4px; padding: 8px; }"
    "QPushButton:pressed { background: #a61e1e; }");
  button_row->addWidget(start_button_);
  button_row->addWidget(stop_button_);
  layout->addLayout(button_row);
  layout->addStretch(1);

  connect(start_button_, &QPushButton::clicked,
          this, &PlannerCsvLoggerPanel::startLogging);
  connect(stop_button_, &QPushButton::clicked,
          this, &PlannerCsvLoggerPanel::stopLogging);

  setStatus("Ready - logging stopped", "#495057");
}

void PlannerCsvLoggerPanel::startLogging()
{
  std_srvs::Trigger service;
  if (!start_client_.call(service)) {
    setStatus("Start failed: /planner_csv_logger/start is unavailable", "#e03131");
    return;
  }
  if (!service.response.success) {
    setStatus(QString::fromStdString(service.response.message), "#e67700");
    return;
  }

  start_button_->setEnabled(false);
  stop_button_->setEnabled(true);
  setStatus(QString::fromStdString(service.response.message), "#2b8a3e");
}

void PlannerCsvLoggerPanel::stopLogging()
{
  std_srvs::Trigger service;
  if (!stop_client_.call(service)) {
    setStatus("Stop failed: /planner_csv_logger/stop is unavailable", "#e03131");
    return;
  }
  if (!service.response.success) {
    setStatus(QString::fromStdString(service.response.message), "#e67700");
    return;
  }

  start_button_->setEnabled(true);
  stop_button_->setEnabled(false);
  setStatus(QString::fromStdString(service.response.message), "#1971c2");
}

void PlannerCsvLoggerPanel::setStatus(const QString & text, const QString & color)
{
  status_label_->setText(text);
  status_label_->setStyleSheet(
    "QLabel { color: " + color + "; background: #f1f3f5; "
    "border: 1px solid #ced4da; border-radius: 4px; padding: 7px; }");
}

}  // namespace trajectory_validator

PLUGINLIB_EXPORT_CLASS(trajectory_validator::PlannerCsvLoggerPanel, rviz::Panel)
