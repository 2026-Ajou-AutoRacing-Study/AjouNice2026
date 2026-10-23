#include "bringup/lanelet_speed_panel.hpp"
#include <pluginlib/class_list_macros.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/view_manager.h>
#include <rviz/view_controller.h>
#include <rviz/properties/property.h>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

namespace bringup
{
LaneletSpeedPanel::LaneletSpeedPanel(QWidget * parent) : rviz::Panel(parent)
{
  commands_ = node_.advertise<std_msgs::String>("/planning/lanelet_speed_editor/command", 10);
  status_ = node_.subscribe<std_msgs::String>("/planning/lanelet_speed_editor/status", 1,
    [this](const std_msgs::String::ConstPtr & msg) {
      Q_EMIT statusReceived(QString::fromStdString(msg->data));
    });
  auto * layout = new QVBoxLayout;
  auto * title = new QLabel("Route B · 제한속도 50 km/h", this);
  layout->addWidget(title);
  auto * help = new QLabel("초록: Route B / 주황: 속도제한 선택\n선호차선 옆의 도로도 선택 가능합니다.\n같은 lanelet을 다시 클릭하면 해제됩니다.", this);
  help->setWordWrap(true);
  layout->addWidget(help);
  auto * views = new QHBoxLayout;
  auto * overview = new QPushButton("Route B 전체", this);
  auto * merge = new QPushButton("합류 구간 확대", this);
  views->addWidget(overview); views->addWidget(merge);
  layout->addLayout(views);
  auto * pick = new QPushButton("Lanelet 선택 (L)", this);
  layout->addWidget(pick);
  lanes_ = new QListWidget(this);
  lanes_->setMinimumHeight(150);
  layout->addWidget(lanes_);
  auto * actions = new QHBoxLayout;
  auto * remove = new QPushButton("선택 해제", this);
  auto * undo = new QPushButton("되돌리기", this);
  actions->addWidget(remove); actions->addWidget(undo);
  layout->addLayout(actions);
  auto * info = new QLabel("클릭 즉시 자동 저장\n다음 Planning·검증 실행부터 적용됩니다.\n기존 도로 제한이 더 낮으면 그 값을 유지합니다.", this);
  info->setWordWrap(true);
  layout->addWidget(info);
  message_ = new QLabel("편집기 연결 대기 중...", this);
  message_->setWordWrap(true);
  layout->addWidget(message_);
  path_ = new QLabel(this);
  path_->setWordWrap(true);
  path_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(path_);
  layout->addStretch();
  setLayout(layout);
  connect(this, &LaneletSpeedPanel::statusReceived, this, &LaneletSpeedPanel::applyStatus);
  const auto view = [this](double x, double y, double scale) {
    if (!vis_manager_) return;
    auto * manager = vis_manager_->getViewManager();
    manager->setCurrentViewControllerType("rviz/TopDownOrtho");
    auto * controller = manager->getCurrent();
    controller->subProp("X")->setValue(x);
    controller->subProp("Y")->setValue(y);
    controller->subProp("Scale")->setValue(scale);
  };
  connect(overview, &QPushButton::clicked, this, [view]() {view(70, 470, 0.75);});
  connect(merge, &QPushButton::clicked, this, [view]() {view(200, 370, 7);});
  connect(pick, &QPushButton::clicked, this, [this]() {
    if (!vis_manager_) return;
    auto * manager = vis_manager_->getToolManager();
    for (int i=0; i<manager->numTools(); ++i) {
      auto * tool = manager->getTool(i);
      if (tool->getClassId() == "bringup/LaneletSpeed") {
        manager->setCurrentTool(tool);
        return;
      }
    }
    auto * tool = manager->addTool("bringup/LaneletSpeed");
    if (tool) manager->setCurrentTool(tool);
  });
  connect(remove, &QPushButton::clicked, this, [this]() {
    auto * item = lanes_->currentItem();
    if (item) command({{"action", "remove"}, {"lanelet_id", item->data(Qt::UserRole).toInt()}});
  });
  connect(undo, &QPushButton::clicked, this, [this]() {command({{"action", "undo"}});});
}
void LaneletSpeedPanel::command(const QJsonObject & value)
{
  std_msgs::String msg;
  msg.data = QJsonDocument(value).toJson(QJsonDocument::Compact).toStdString();
  commands_.publish(msg);
}
void LaneletSpeedPanel::applyStatus(const QString & value)
{
  const auto document = QJsonDocument::fromJson(value.toUtf8());
  if (!document.isObject()) return;
  const auto object = document.object();
  const auto * old = lanes_->currentItem();
  const int selected = old ? old->data(Qt::UserRole).toInt() : -1;
  lanes_->clear();
  for (const auto & entry : object.value("selected").toArray()) {
    const auto lane = entry.toObject();
    const int id = lane.value("lanelet_id").toInt();
    auto * item = new QListWidgetItem(QString("%1  →  %2 km/h").arg(id).arg(lane.value("speed_kmh").toDouble()), lanes_);
    item->setData(Qt::UserRole, id);
    if (id == selected || id == object.value("last_lanelet_id").toInt()) lanes_->setCurrentItem(item);
  }
  message_->setText(object.value("message").toString());
  path_->setText("저장 파일:\n"+object.value("save_file").toString());
}
}
PLUGINLIB_EXPORT_CLASS(bringup::LaneletSpeedPanel, rviz::Panel)
