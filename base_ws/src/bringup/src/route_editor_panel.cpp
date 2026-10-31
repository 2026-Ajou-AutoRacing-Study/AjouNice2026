#include "bringup/route_editor_panel.hpp"
#include <pluginlib/class_list_macros.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>
#include <QComboBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

namespace bringup
{
RouteEditorPanel::RouteEditorPanel(QWidget * parent) : rviz::Panel(parent)
{
  commands_ = node_.advertise<std_msgs::String>("/planning/route_editor/command", 10);
  status_ = node_.subscribe<std_msgs::String>("/planning/route_editor/status", 1,
    [this](const std_msgs::String::ConstPtr & msg) {
      Q_EMIT statusReceived(QString::fromStdString(msg->data));
    });
  auto * layout = new QVBoxLayout;
  route_ = new QComboBox(this);
  route_->addItem("Route A", "route_a");
  route_->addItem("Route B", "route_b");
  layout->addWidget(route_);
  auto * legend = new QLabel("원본 A: 파랑 / B: 초록\n편집 미리보기: 주황 / 연결 불가: 빨강", this);
  legend->setToolTip("미리보기는 Lanelet2 도로 연결 기준입니다. 실제 경로는 Planning 적용 후 Mission Route에서 확인합니다.");
  layout->addWidget(legend);
  auto * add = new QPushButton("클릭 추가 (P)", this);
  layout->addWidget(add);
  auto * help = new QLabel("도로를 클릭하면 순서에 맞게 추가됩니다.\nInteract로 점을 드래그해 이동하세요.", this);
  help->setWordWrap(true);
  layout->addWidget(help);
  points_ = new QListWidget(this);
  points_->setMinimumHeight(110);
  points_->setMaximumHeight(210);
  layout->addWidget(points_);
  auto * actions = new QHBoxLayout;
  auto * remove = new QPushButton("선택 삭제", this);
  auto * undo = new QPushButton("되돌리기", this);
  auto * reset = new QPushButton("원본 복원", this);
  actions->addWidget(remove); actions->addWidget(undo); actions->addWidget(reset);
  layout->addLayout(actions);
  path_ = new QLineEdit(this);
  path_->setPlaceholderText("편집본 저장 경로 (.yaml)");
  layout->addWidget(path_);
  auto * files = new QHBoxLayout;
  auto * save = new QPushButton("저장", this);
  auto * load = new QPushButton("불러오기", this);
  files->addWidget(save); files->addWidget(load);
  layout->addLayout(files);
  apply_ = new QPushButton("Planning에 경로 적용", this);
  apply_->setToolTip("실행 중인 Mission Planner에 경유점과 목적지를 요청합니다.");
  layout->addWidget(apply_);
  message_ = new QLabel("경로 편집기 연결 대기 중...", this);
  message_->setWordWrap(true);
  layout->addWidget(message_);
  setLayout(layout);
  connect(this, &RouteEditorPanel::statusReceived, this, &RouteEditorPanel::applyStatus);
  connect(route_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
    command({{"action", "select"}, {"route", route_->currentData().toString()}});
  });
  connect(add, &QPushButton::clicked, this, [this]() {
    if (!vis_manager_) return;
    auto * manager = vis_manager_->getToolManager();
    for (int i = 0; i < manager->numTools(); ++i) {
      auto * tool = manager->getTool(i);
      if (tool->getClassId() == "bringup/RoutePoint") {
        manager->setCurrentTool(tool);
        return;
      }
    }
    auto * tool = manager->addTool("bringup/RoutePoint");
    if (tool) manager->setCurrentTool(tool);
  });
  connect(remove, &QPushButton::clicked, this, [this]() {
    command({{"action", "delete"}, {"index", points_->currentRow()}});
  });
  connect(undo, &QPushButton::clicked, this, [this]() { command({{"action", "undo"}}); });
  connect(reset, &QPushButton::clicked, this, [this]() { command({{"action", "reset"}}); });
  connect(save, &QPushButton::clicked, this, [this]() {
    QString path = path_->text().trimmed();
    if (path.isEmpty()) path = QFileDialog::getSaveFileName(this, "경로 편집본 저장", "", "YAML (*.yaml)");
    if (!path.isEmpty()) command({{"action", "save"}, {"path", path}});
  });
  connect(load, &QPushButton::clicked, this, [this]() {
    const QString path = QFileDialog::getOpenFileName(this, "경로 편집본 불러오기", path_->text(), "YAML (*.yaml *.yml)");
    if (!path.isEmpty()) command({{"action", "load"}, {"path", path}});
  });
  connect(apply_, &QPushButton::clicked, this, [this]() { command({{"action", "apply"}}); });
}

void RouteEditorPanel::command(const QJsonObject & value)
{
  std_msgs::String msg;
  msg.data = QJsonDocument(value).toJson(QJsonDocument::Compact).toStdString();
  commands_.publish(msg);
}

void RouteEditorPanel::applyStatus(const QString & value)
{
  const auto document = QJsonDocument::fromJson(value.toUtf8());
  if (!document.isObject()) return;
  const auto object = document.object();
  const QSignalBlocker blocker(route_);
  route_->setCurrentIndex(object.value("route").toString() == "route_b" ? 1 : 0);
  int selected = object.value("selected_index").toInt(-1);
  if (selected < 0) selected = points_->currentRow();
  points_->clear();
  int index = 0;
  for (const auto & value : object.value("points").toArray()) {
    const auto point = value.toObject();
    points_->addItem(QString("%1  %2  [L%3]").arg(index++)
      .arg(point.value("label").toString()).arg(point.value("lanelet_id").toInt()));
  }
  if (selected >= 0 && selected < points_->count()) points_->setCurrentRow(selected);
  if (!path_->hasFocus()) path_->setText(object.value("save_file").toString());
  apply_->setEnabled(!object.value("busy").toBool());
  message_->setText(object.value("message").toString());
}
}  // namespace bringup
PLUGINLIB_EXPORT_CLASS(bringup::RouteEditorPanel, rviz::Panel)
