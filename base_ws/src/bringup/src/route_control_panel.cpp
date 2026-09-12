#include "bringup/route_control_panel.hpp"

#include <competition_route_requester/RequestRoute.h>
#include <pluginlib/class_list_macros.h>

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace bringup
{

RouteControlPanel::RouteControlPanel(QWidget * parent)
: rviz::Panel(parent), status_label_(nullptr), request_button_(nullptr)
{
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>(
    "route_request_service", request_service_name_,
    "/competition_route_requester/request");
  std::string status_topic;
  private_node_handle.param<std::string>(
    "route_request_status_topic", status_topic,
    "/competition_route_requester/request_status");

  request_client_ = node_handle_.serviceClient<competition_route_requester::RequestRoute>(
    request_service_name_, false);
  status_subscriber_ = node_handle_.subscribe(
    status_topic, 10, &RouteControlPanel::onStatus, this);

  auto * layout = new QVBoxLayout;
  auto * title = new QLabel("Competition Route", this);
  QFont title_font = title->font();
  title_font.setBold(true);
  title->setFont(title_font);
  layout->addWidget(title);

  auto * instruction = new QLabel(
    "Place and stop the vehicle at the official start, then request Route A. "
    "Press again there to refresh it; Route B/fallback remains automatic.", this);
  instruction->setWordWrap(true);
  layout->addWidget(instruction);

  request_button_ = new QPushButton("REQUEST ROUTE A", this);
  request_button_->setStyleSheet(
    "QPushButton { background-color: #2563a8; color: white; font-weight: bold; "
    "padding: 8px; }");
  layout->addWidget(request_button_);

  status_label_ = new QLabel("Ready for manual Route A request", this);
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);
  setLayout(layout);

  connect(request_button_, &QPushButton::clicked, this, &RouteControlPanel::requestRouteA);
  connect(this, &RouteControlPanel::statusReceived, this, &RouteControlPanel::applyStatus);
}

void RouteControlPanel::requestRouteA()
{
  request_button_->setEnabled(false);
  status_label_->setText("Requesting Route A...");

  if (!request_client_.waitForExistence(ros::Duration(0.5))) {
    status_label_->setText(
      "ERROR: route requester service is unavailable. Start planning with requester enabled.");
    request_button_->setEnabled(true);
    return;
  }

  competition_route_requester::RequestRoute request;
  request.request.route_key = "route_a";
  if (!request_client_.call(request)) {
    status_label_->setText("ERROR: Route A request transport failed");
    request_button_->setEnabled(true);
    return;
  }

  if (!request.response.accepted) {
    status_label_->setText(
      "REJECTED: " + QString::fromStdString(request.response.detail));
    request_button_->setEnabled(true);
    return;
  }

  status_label_->setText("Route A queued; waiting for route and trajectory verification...");
}

void RouteControlPanel::onStatus(const std_msgs::String::ConstPtr & message)
{
  Q_EMIT statusReceived(QString::fromStdString(message->data));
}

void RouteControlPanel::applyStatus(const QString & status)
{
  status_label_->setText(status);
  request_button_->setEnabled(!status.contains("\"busy\": true"));
}

}  // namespace bringup

PLUGINLIB_EXPORT_CLASS(bringup::RouteControlPanel, rviz::Panel)
