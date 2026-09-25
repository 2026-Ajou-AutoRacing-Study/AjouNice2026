#ifndef BRINGUP__ROSBAG_RECORDER_PANEL_HPP_
#define BRINGUP__ROSBAG_RECORDER_PANEL_HPP_

#include <rviz/panel.h>

#include <QProcess>

#include <string>

class QLabel;
class QLineEdit;
class QPushButton;

namespace bringup
{

class RosbagRecorderPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit RosbagRecorderPanel(QWidget * parent = nullptr);
  ~RosbagRecorderPanel() override;

private Q_SLOTS:
  void startRecording();
  void stopRecording();
  void recorderFinished(int exit_code, QProcess::ExitStatus exit_status);
  void discardRecorderOutput();

private:
  void stopRecorderForShutdown();
  QString makeOutputPath() const;
  void nameCompletedBag();

  QLineEdit * output_directory_edit_;
  QLabel * status_label_;
  QPushButton * start_button_;
  QPushButton * stop_button_;
  QPushButton * rename_button_;
  QProcess * recorder_;
  QString topics_;
  QString active_output_path_;
  QString recording_stamp_;
  QString completed_output_path_;
  QString completed_stamp_;
  QString last_scenario_name_;
  bool stop_requested_{false};
  bool shutting_down_{false};
};

}  // namespace bringup

#endif  // BRINGUP__ROSBAG_RECORDER_PANEL_HPP_
