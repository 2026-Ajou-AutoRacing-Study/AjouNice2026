#include "bringup/rosbag_recorder_panel.hpp"
#include "bringup/rosbag_filename.hpp"

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/master.h>

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QFile>
#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QInputDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QRegExp>
#include <QStringList>
#include <QVBoxLayout>

#include <signal.h>

#include <cstdlib>
#include <string>

namespace bringup
{
namespace
{
// Cover conventional payload/display topic names that appear after START.
// Keep semantic perception/planning data and raw LiDAR packets eligible.
constexpr const char * kExcludedTopicPattern =
  ".*/(image|image_raw|image_rect|image_rect_color|image_color|image_mono|"
  "compressed|compressedDepth|theora)(/.*)?|"
  "/sensor/lidar1|.*/(points|points_raw|points_filtered|pointcloud|pointcloud_raw)|"
  ".*/(visualization|markers|marker|virtual_wall|virtual_walls)(/.*)?|"
  "/app/perc/(vis_[^/]+|jsk_all_track)|/planning/debug/final_trajectory/path";

QString recordingExclusionPattern(
  const ros::master::V_TopicInfo & topics, const bool record_morai_gt)
{
  QStringList patterns{QString::fromLatin1(kExcludedTopicPattern)};
  if (!record_morai_gt) patterns.append("/Ego_topic|/Object_topic");
  for (const auto & topic : topics) {
    // Type discovery also covers nonstandard names at START. PointCloud types
    // are derived point sets; velodyne_msgs/VelodyneScan packets stay recorded.
    if (topic.datatype == "sensor_msgs/Image" ||
        topic.datatype == "sensor_msgs/CompressedImage" ||
        topic.datatype == "theora_image_transport/Packet" ||
        topic.datatype == "sensor_msgs/PointCloud" ||
        topic.datatype == "sensor_msgs/PointCloud2" ||
        topic.datatype.find("visualization_msgs/") == 0 ||
        topic.datatype.find("rviz_2d_overlay_msgs/") == 0 ||
        topic.datatype.find("jsk_rviz_plugins/") == 0) {
      patterns.append(QRegExp::escape(QString::fromStdString(topic.name)));
    }
  }
  return "(" + patterns.join("|") + ")";
}

QString defaultOutputDirectory()
{
  const char * root = std::getenv("AJOUNICE_ROOT");
  if (root != nullptr && root[0] != '\0') {
    return QDir(QString::fromLocal8Bit(root)).filePath("planner_logs/bags");
  }
  return QDir::home().filePath(".ros/bags");
}
}  // namespace

RosbagRecorderPanel::RosbagRecorderPanel(QWidget * parent)
: rviz::Panel(parent), recorder_(new QProcess(this))
{
  ros::NodeHandle private_nh("~");
  std::string output_directory = defaultOutputDirectory().toStdString();
  private_nh.param("rosbag_output_directory", output_directory, output_directory);

  auto * main_layout = new QVBoxLayout;
  main_layout->setContentsMargins(6, 6, 6, 6);

  auto * title = new QLabel("Rosbag (no images / clouds / visuals)", this);
  QFont title_font = title->font();
  title_font.setBold(true);
  title->setFont(title_font);
  main_layout->addWidget(title);

  output_directory_edit_ = new QLineEdit(QString::fromStdString(output_directory), this);
  output_directory_edit_->setToolTip("Directory where timestamped .bag files are saved");
  main_layout->addWidget(output_directory_edit_);

  auto * button_layout = new QHBoxLayout;
  start_button_ = new QPushButton("START", this);
  stop_button_ = new QPushButton("STOP", this);
  start_button_->setStyleSheet(
    "QPushButton { background-color: #258b45; color: white; font-weight: bold; }");
  stop_button_->setStyleSheet(
    "QPushButton { background-color: #b53a3a; color: white; font-weight: bold; }");
  stop_button_->setEnabled(false);
  button_layout->addWidget(start_button_);
  button_layout->addWidget(stop_button_);
  main_layout->addLayout(button_layout);

  rename_button_ = new QPushButton(QString::fromUtf8("저장한 백 이름 변경"), this);
  rename_button_->setEnabled(false);
  main_layout->addWidget(rename_button_);

  status_label_ = new QLabel("Ready", this);
  status_label_->setWordWrap(true);
  main_layout->addWidget(status_label_);
  setLayout(main_layout);

  connect(start_button_, &QPushButton::clicked, this, &RosbagRecorderPanel::startRecording);
  connect(stop_button_, &QPushButton::clicked, this, &RosbagRecorderPanel::stopRecording);
  connect(rename_button_, &QPushButton::clicked, this, &RosbagRecorderPanel::nameCompletedBag);
  connect(
    recorder_, static_cast<void (QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished),
    this, &RosbagRecorderPanel::recorderFinished);
  connect(
    recorder_, &QProcess::readyReadStandardOutput, this,
    &RosbagRecorderPanel::discardRecorderOutput);
  recorder_->setProcessChannelMode(QProcess::MergedChannels);
}

RosbagRecorderPanel::~RosbagRecorderPanel()
{
  shutting_down_ = true;
  stopRecorderForShutdown();
}

QString RosbagRecorderPanel::makeOutputPath() const
{
  QDir directory(output_directory_edit_->text().trimmed());
  const QString stamp = recording_stamp_;
  QString path = directory.filePath("planning_rviz_" + stamp + ".bag");
  int suffix = 1;
  while (QFileInfo::exists(path) || QFileInfo::exists(path + ".active")) {
    path = directory.filePath(
      QString("planning_rviz_%1_%2.bag").arg(stamp).arg(suffix++));
  }
  return path;
}

void RosbagRecorderPanel::startRecording()
{
  if (recorder_->state() != QProcess::NotRunning) {
    return;
  }

  QDir output_directory(output_directory_edit_->text().trimmed());
  if (!output_directory.mkpath(".")) {
    status_label_->setText("ERROR: cannot create output directory");
    return;
  }

  ros::master::V_TopicInfo published_topics;
  if (!ros::master::getTopics(published_topics)) {
    status_label_->setText("ERROR: cannot query ROS topics for recording exclusions");
    return;
  }

  ros::NodeHandle private_nh("~");
  bool record_morai_gt = true;
  private_nh.param("record_morai_gt", record_morai_gt, true);

  recording_stamp_ = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
  active_output_path_ = makeOutputPath();
  QStringList arguments{
    "record", "--lz4", "--buffsize=1024", "--all",
    "--exclude", recordingExclusionPattern(published_topics, record_morai_gt), "-O", active_output_path_};

  stop_requested_ = false;
  recorder_->start("rosbag", arguments);
  if (!recorder_->waitForStarted(3000)) {
    status_label_->setText("ERROR: failed to start rosbag");
    active_output_path_.clear();
    return;
  }

  output_directory_edit_->setEnabled(false);
  start_button_->setEnabled(false);
  stop_button_->setEnabled(true);
  rename_button_->setEnabled(false);
  status_label_->setText("Recording: " + QFileInfo(active_output_path_).fileName());
}

void RosbagRecorderPanel::stopRecording()
{
  if (recorder_->state() == QProcess::NotRunning) {
    return;
  }
  stop_requested_ = true;
  stop_button_->setEnabled(false);
  status_label_->setText("Finalizing bag...");
  ::kill(static_cast<pid_t>(recorder_->processId()), SIGINT);
}

void RosbagRecorderPanel::recorderFinished(
  int exit_code, QProcess::ExitStatus exit_status)
{
  stop_button_->setEnabled(false);

  if (stop_requested_ && exit_status == QProcess::NormalExit && exit_code == 0 &&
      QFileInfo::exists(active_output_path_) &&
      !QFileInfo::exists(active_output_path_ + ".active")) {
    completed_output_path_ = active_output_path_;
    completed_stamp_ = recording_stamp_;
    status_label_->setText("Saved: " + completed_output_path_);
  } else if (shutting_down_ && QFileInfo::exists(active_output_path_)) {
    status_label_->setText("Saved: " + active_output_path_);
  } else {
    status_label_->setText(
      QString("Recorder stopped unexpectedly (exit %1)").arg(exit_code));
  }
  stop_requested_ = false;
  output_directory_edit_->setEnabled(true);
  start_button_->setEnabled(true);
  rename_button_->setEnabled(!shutting_down_ && QFileInfo::exists(completed_output_path_));
  rename_button_->setToolTip(completed_output_path_);
}

void RosbagRecorderPanel::nameCompletedBag()
{
  if (recorder_->state() != QProcess::NotRunning ||
      !QFileInfo::exists(completed_output_path_) ||
      QFileInfo::exists(completed_output_path_ + ".active")) {
    status_label_->setText("Cannot rename: completed bag is unavailable");
    return;
  }
  bool accepted = false;
  const QString entered = QInputDialog::getText(
    this, QString::fromUtf8("로스백 시나리오 이름"),
    QString::fromUtf8("시나리오 약칭 (예: scooter5, scooter5_bevfusion)\n"
                      "취소하면 현재 파일명으로 보관합니다."),
    QLineEdit::Normal, last_scenario_name_, &accepted);
  const QString scenario = normalizeBagScenario(entered);
  if (!accepted || scenario.isEmpty()) {
    status_label_->setText("Saved (name unchanged): " + completed_output_path_);
    return;
  }
  last_scenario_name_ = scenario;
  const QDir directory = QFileInfo(completed_output_path_).dir();
  const QString requested = directory.filePath(
    "plan_rviz_" + scenario + "_" + completed_stamp_ + ".bag");
  if (requested == completed_output_path_) {
    status_label_->setText("Saved (name unchanged): " + completed_output_path_);
    return;
  }
  const QString destination = completedBagPath(directory, scenario, completed_stamp_);
  if (!QFile::rename(completed_output_path_, destination)) {
    status_label_->setText("Saved; rename failed. Original kept: " + completed_output_path_);
    return;
  }
  completed_output_path_ = destination;
  rename_button_->setToolTip(completed_output_path_);
  status_label_->setText("Saved: " + completed_output_path_);
}

void RosbagRecorderPanel::discardRecorderOutput()
{
  recorder_->readAllStandardOutput();
}

void RosbagRecorderPanel::stopRecorderForShutdown()
{
  if (recorder_->state() == QProcess::NotRunning) {
    return;
  }
  ::kill(static_cast<pid_t>(recorder_->processId()), SIGINT);
  if (!recorder_->waitForFinished(3000)) {
    recorder_->terminate();
    if (!recorder_->waitForFinished(1000)) {
      recorder_->kill();
      recorder_->waitForFinished(1000);
    }
  }
}

}  // namespace bringup

PLUGINLIB_EXPORT_CLASS(bringup::RosbagRecorderPanel, rviz::Panel)
