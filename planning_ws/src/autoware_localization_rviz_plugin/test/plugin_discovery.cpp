#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rviz/display.h>

#include <set>
#include <string>
#include <vector>

TEST(LocalizationRvizPlugin, declares_all_ros1_displays)
{
  pluginlib::ClassLoader<rviz::Display> loader(
    "rviz", "rviz::Display");
  const std::vector<std::string> classes = loader.getDeclaredClasses();
  const std::set<std::string> declared(classes.begin(), classes.end());
  EXPECT_NE(declared.find("rviz_plugins::PoseHistory"), declared.end());
  EXPECT_NE(declared.find("rviz_plugins::PoseHistoryFootprint"), declared.end());
  EXPECT_NE(declared.find("rviz_plugins::PoseWithCovarianceHistory"), declared.end());
  EXPECT_NE(declared.find("rviz_plugins::ColoredPoseWithCovarianceHistory"), declared.end());
}
