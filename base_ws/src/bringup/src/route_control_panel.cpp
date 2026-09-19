#include "bringup/route_control_panel.hpp"

#include <pluginlib/class_list_macros.h>

#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QLabel>
#include <QVBoxLayout>

namespace bringup
{

RouteControlPanel::RouteControlPanel(QWidget * parent)
: rviz::Panel(parent),
  route_a_indicator_(nullptr),
  route_b_indicator_(nullptr),
  status_label_(nullptr)
{
  ros::NodeHandle private_node_handle("~");
  std::string status_topic;
  private_node_handle.param<std::string>(
    "route_request_status_topic", status_topic,
    "/competition_route_requester/request_status");

  status_subscriber_ = node_handle_.subscribe(
    status_topic, 10, &RouteControlPanel::onStatus, this);

  auto * layout = new QVBoxLayout;
  auto * title = new QLabel("Competition Route", this);
  QFont title_font = title->font();
  title_font.setBold(true);
  title->setFont(title_font);
  layout->addWidget(title);

  auto * indicators = new QHBoxLayout;
  route_a_indicator_ = new QLabel("ROUTE A", this);
  route_b_indicator_ = new QLabel("ROUTE B", this);
  route_a_indicator_->setAlignment(Qt::AlignCenter);
  route_b_indicator_->setAlignment(Qt::AlignCenter);
  route_a_indicator_->setMinimumHeight(38);
  route_b_indicator_->setMinimumHeight(38);
  indicators->addWidget(route_a_indicator_);
  indicators->addWidget(route_b_indicator_);
  layout->addLayout(indicators);

  status_label_ = new QLabel("Waiting for verified route status...", this);
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);
  setLayout(layout);

  connect(this, &RouteControlPanel::statusReceived, this, &RouteControlPanel::applyStatus);
  setActiveRoute(QString());
}

void RouteControlPanel::onStatus(const std_msgs::String::ConstPtr & message)
{
  Q_EMIT statusReceived(QString::fromStdString(message->data));
}

void RouteControlPanel::applyStatus(const QString & status)
{
  QJsonParseError parse_error;
  const QJsonDocument document = QJsonDocument::fromJson(status.toUtf8(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject()) {
    setActiveRoute(QString());
    status_label_->setText("Invalid route status: " + status);
    return;
  }

  const QJsonObject object = document.object();
  const QString active_route = object.value("verified_active").toString();
  const QString phase = object.value("phase").toString("UNKNOWN");
  const QString detail = object.value("detail").toString();

  setActiveRoute(active_route);
  status_label_->setText(detail.isEmpty() ? phase : phase + ": " + detail);
}

void RouteControlPanel::setActiveRoute(const QString & route_key)
{
  const QString active_style =
    "QLabel { background-color: #22c55e; color: #07130b; border: 2px solid #86efac; "
    "border-radius: 5px; font-weight: bold; padding: 7px; }";
  const QString inactive_style =
    "QLabel { background-color: #30343b; color: #8b929c; border: 2px solid #4b515b; "
    "border-radius: 5px; font-weight: bold; padding: 7px; }";

  route_a_indicator_->setStyleSheet(route_key == "route_a" ? active_style : inactive_style);
  route_b_indicator_->setStyleSheet(route_key == "route_b" ? active_style : inactive_style);
}

}  // namespace bringup

PLUGINLIB_EXPORT_CLASS(bringup::RouteControlPanel, rviz::Panel)
