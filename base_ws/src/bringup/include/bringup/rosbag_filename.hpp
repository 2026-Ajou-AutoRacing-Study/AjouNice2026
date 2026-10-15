#ifndef BRINGUP__ROSBAG_FILENAME_HPP_
#define BRINGUP__ROSBAG_FILENAME_HPP_

#include <QDir>
#include <QFileInfo>
#include <QRegularExpression>
#include <QString>

namespace bringup
{
inline QString normalizeBagScenario(QString name)
{
  name = name.trimmed().toLower();
  name.replace(QRegularExpression("[^a-z0-9]+"), "_");
  name.replace(QRegularExpression("^_+|_+$"), "");
  name.replace(QRegularExpression("^scooter_*(?:pos(?:ition)?_*)?(\\d+)(?=_|$)"),
               "scooter\\1");
  return name;
}

inline QString completedBagPath(
  const QDir & directory, const QString & scenario, const QString & stamp)
{
  const QString stem = "plan_rviz_" + scenario + "_" + stamp;
  QString path = directory.filePath(stem + ".bag");
  int suffix = 1;
  while (QFileInfo::exists(path) || QFileInfo::exists(path + ".active")) {
    path = directory.filePath(stem + "_" + QString::number(suffix++) + ".bag");
  }
  return path;
}
}  // namespace bringup

#endif  // BRINGUP__ROSBAG_FILENAME_HPP_
