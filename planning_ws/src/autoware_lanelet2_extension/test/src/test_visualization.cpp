#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <ros/ros.h>

namespace
{
lanelet::ConstLanelet makeLanelet()
{
  using lanelet::LineString3d;
  using lanelet::Point3d;
  const auto left = LineString3d(
    lanelet::utils::getId(),
    {Point3d(lanelet::utils::getId(), 0.0, 0.0, 0.0),
     Point3d(lanelet::utils::getId(), 0.0, 2.0, 0.0)});
  const auto right = LineString3d(
    lanelet::utils::getId(),
    {Point3d(lanelet::utils::getId(), 2.0, 0.0, 0.0),
     Point3d(lanelet::utils::getId(), 2.0, 2.0, 0.0)});
  lanelet::Lanelet lanelet(lanelet::utils::getId(), left, right);
  lanelet.setCenterline(LineString3d(
    lanelet::utils::getId(),
    {Point3d(lanelet::utils::getId(), 1.0, 0.0, 0.0),
     Point3d(lanelet::utils::getId(), 1.0, 2.0, 0.0)}));
  return lanelet;
}

std_msgs::ColorRGBA color()
{
  std_msgs::ColorRGBA value;
  value.r = 0.25F;
  value.g = 0.5F;
  value.b = 0.75F;
  value.a = 1.0F;
  return value;
}
}  // namespace

TEST(Visualization, ZeroStampInitializerPreservesMarkerDefaults)
{
  visualization_msgs::Marker marker;
  lanelet::visualization::initLineStringMarker(&marker, "map", "boundary", color());
  EXPECT_EQ("map", marker.header.frame_id);
  EXPECT_EQ(ros::Time(0), marker.header.stamp);
  EXPECT_EQ("boundary", marker.ns);
  EXPECT_EQ(visualization_msgs::Marker::ADD, marker.action);
  EXPECT_EQ(visualization_msgs::Marker::TRIANGLE_LIST, marker.type);
  EXPECT_DOUBLE_EQ(1.0, marker.pose.orientation.w);
  EXPECT_FLOAT_EQ(0.25F, marker.color.r);
}

TEST(Visualization, LaneletIdMarkerUsesIndependentSystemTime)
{
  ros::Time::init();
  const ros::Time simulated_time(123, 456);
  ros::Time::setNow(simulated_time);
  const auto before = ros::WallTime::now();
  const auto markers = lanelet::visualization::generateLaneletIdMarker(
    lanelet::ConstLanelets{makeLanelet()}, color(), "lanelet_id", 0.5);
  const auto after = ros::WallTime::now();
  ASSERT_EQ(1U, markers.markers.size());
  EXPECT_NE(simulated_time, markers.markers.front().header.stamp);
  EXPECT_GE(markers.markers.front().header.stamp.toSec(), before.toSec());
  EXPECT_LE(markers.markers.front().header.stamp.toSec(), after.toSec());
  EXPECT_EQ("map", markers.markers.front().header.frame_id);
  EXPECT_EQ("lanelet_id", markers.markers.front().ns);
  EXPECT_EQ(visualization_msgs::Marker::TEXT_VIEW_FACING, markers.markers.front().type);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "autoware_lanelet2_extension_visualization_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
