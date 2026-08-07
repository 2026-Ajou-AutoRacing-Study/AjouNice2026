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
  bag_start_client_ = node_handle_.serviceClient<std_srvs::Trigger>(
    "/planner_bag_recorder/start");
  bag_stop_client_ = node_handle_.serviceClient<std_srvs::Trigger>(
    "/planner_bag_recorder/stop");

  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(8);

  auto * title = new QLabel("Planner Data Recorder", this);
  title->setStyleSheet("QLabel { font: 700 15px 'DejaVu Sans'; }");
  layout->addWidget(title);

  auto * csv_title = new QLabel("CSV diagnostics", this);
  csv_title->setStyleSheet("QLabel { font-weight: 700; color: #343a40; }");
  layout->addWidget(csv_title);

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

  auto * bag_title = new QLabel("Planner debug bag", this);
  bag_title->setStyleSheet(
    "QLabel { font-weight: 700; color: #343a40; margin-top: 8px; }");
  layout->addWidget(bag_title);

  bag_status_label_ = new QLabel("Ready - bag recording stopped", this);
  bag_status_label_->setWordWrap(true);
  bag_status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  bag_status_label_->setMinimumHeight(42);
  layout->addWidget(bag_status_label_);

  auto * bag_button_row = new QHBoxLayout();
  bag_button_row->setSpacing(8);
  bag_start_button_ = new QPushButton("Start Bag", this);
  bag_stop_button_ = new QPushButton("Stop Bag", this);
  bag_start_button_->setMinimumHeight(42);
  bag_stop_button_->setMinimumHeight(42);
  bag_start_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  bag_stop_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  bag_start_button_->setStyleSheet(
    "QPushButton { background: #2b8a3e; color: white; font-weight: 700; "
    "border-radius: 4px; padding: 8px; }"
    "QPushButton:pressed { background: #237032; }");
  bag_stop_button_->setStyleSheet(
    "QPushButton { background: #c92a2a; color: white; font-weight: 700; "
    "border-radius: 4px; padding: 8px; }"
    "QPushButton:pressed { background: #a61e1e; }");
  bag_button_row->addWidget(bag_start_button_);
  bag_button_row->addWidget(bag_stop_button_);
  layout->addLayout(bag_button_row);
  layout->addStretch(1);

  connect(start_button_, &QPushButton::clicked,
          this, &PlannerCsvLoggerPanel::startLogging);
  connect(stop_button_, &QPushButton::clicked,
          this, &PlannerCsvLoggerPanel::stopLogging);
  connect(bag_start_button_, &QPushButton::clicked,
          this, &PlannerCsvLoggerPanel::startBagRecording);
  connect(bag_stop_button_, &QPushButton::clicked,
          this, &PlannerCsvLoggerPanel::stopBagRecording);

  stop_button_->setEnabled(false);
  bag_stop_button_->setEnabled(false);
  setStatus(status_label_, "Ready - logging stopped", "#495057");
  setStatus(bag_status_label_, "Ready - bag recording stopped", "#495057");
}

void PlannerCsvLoggerPanel::startLogging()
{
  std_srvs::Trigger service;
  if (!start_client_.call(service)) {
    setStatus(status_label_, "Start failed: /planner_csv_logger/start is unavailable", "#e03131");
    return;
  }
  if (!service.response.success) {
    setStatus(status_label_, QString::fromStdString(service.response.message), "#e67700");
    return;
  }

  start_button_->setEnabled(false);
  stop_button_->setEnabled(true);
  setStatus(status_label_, QString::fromStdString(service.response.message), "#2b8a3e");
}

void PlannerCsvLoggerPanel::stopLogging()
{
  std_srvs::Trigger service;
  if (!stop_client_.call(service)) {
    setStatus(status_label_, "Stop failed: /planner_csv_logger/stop is unavailable", "#e03131");
    return;
  }
  if (!service.response.success) {
    setStatus(status_label_, QString::fromStdString(service.response.message), "#e67700");
    return;
  }

  start_button_->setEnabled(true);
  stop_button_->setEnabled(false);
  setStatus(status_label_, QString::fromStdString(service.response.message), "#1971c2");
}

void PlannerCsvLoggerPanel::startBagRecording()
{
  std_srvs::Trigger service;
  if (!bag_start_client_.call(service)) {
    setStatus(
      bag_status_label_,
      "Start failed: /planner_bag_recorder/start is unavailable", "#e03131");
    return;
  }
  if (!service.response.success) {
    setStatus(
      bag_status_label_, QString::fromStdString(service.response.message), "#e67700");
    return;
  }

  bag_start_button_->setEnabled(false);
  bag_stop_button_->setEnabled(true);
  setStatus(
    bag_status_label_, QString::fromStdString(service.response.message), "#1971c2");
}

void PlannerCsvLoggerPanel::stopBagRecording()
{
  std_srvs::Trigger service;
  if (!bag_stop_client_.call(service)) {
    setStatus(
      bag_status_label_,
      "Stop failed: /planner_bag_recorder/stop is unavailable", "#e03131");
    return;
  }
  if (!service.response.success) {
    setStatus(
      bag_status_label_, QString::fromStdString(service.response.message), "#e67700");
    return;
  }

  bag_start_button_->setEnabled(true);
  bag_stop_button_->setEnabled(false);
  setStatus(
    bag_status_label_, QString::fromStdString(service.response.message), "#2b8a3e");
}

void PlannerCsvLoggerPanel::setStatus(
  QLabel * label, const QString & text, const QString & color)
{
  label->setText(text);
  label->setStyleSheet(
    "QLabel { color: " + color + "; background: #f1f3f5; "
    "border: 1px solid #ced4da; border-radius: 4px; padding: 7px; }");
}

}  // namespace trajectory_validator

PLUGINLIB_EXPORT_CLASS(trajectory_validator::PlannerCsvLoggerPanel, rviz::Panel)
