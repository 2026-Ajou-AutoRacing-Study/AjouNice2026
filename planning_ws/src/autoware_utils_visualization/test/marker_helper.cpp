#include "autoware_utils_visualization/marker_helper.hpp"

#include <gtest/gtest.h>

namespace auv = autoware_utils_visualization;

TEST(MarkerHelper, ScalarFieldsArePreserved)
{
  const auto p = auv::create_marker_position(0.1, 0.2, 0.3);
  const auto q = auv::create_marker_orientation(0.1, 0.2, 0.3, 0.4);
  const auto s = auv::create_marker_scale(0.1, 0.2, 0.3);
  const auto c = auv::create_marker_color(0.1F, 0.2F, 0.3F, 0.4F);
  EXPECT_DOUBLE_EQ(p.z, 0.3); EXPECT_DOUBLE_EQ(q.w, 0.4); EXPECT_DOUBLE_EQ(s.y, 0.2);
  EXPECT_FLOAT_EQ(c.a, 0.4F);
}

TEST(MarkerHelper, DefaultAndDeleteMarkerContract)
{
  const ros::Time stamp(12345, 67890);
  const auto marker = auv::create_default_marker(
    "frame", stamp, "ns", 99, visualization_msgs::Marker::CUBE,
    auv::create_marker_scale(0.1, 0.2, 0.3), auv::create_marker_color(0.1F, 0.2F, 0.3F, 0.4F));
  EXPECT_EQ(marker.header.stamp, stamp); EXPECT_EQ(marker.header.frame_id, "frame");
  EXPECT_EQ(marker.ns, "ns"); EXPECT_EQ(marker.id, 99); EXPECT_EQ(marker.action, visualization_msgs::Marker::ADD);
  EXPECT_EQ(marker.type, visualization_msgs::Marker::CUBE); EXPECT_TRUE(marker.frame_locked);
  EXPECT_DOUBLE_EQ(marker.lifetime.toSec(), auv::MARKER_LIFETIME);
  const auto deleted = auv::create_deleted_default_marker(stamp, "ns", 99);
  EXPECT_EQ(deleted.action, visualization_msgs::Marker::DELETE); EXPECT_EQ(deleted.header.stamp, stamp);
}

TEST(MarkerHelper, AppendPreservesOrReplacesTimestamp)
{
  visualization_msgs::MarkerArray destination, additional;
  destination.markers.resize(1); destination.markers[0].id = 10;
  additional.markers.resize(2); additional.markers[0].id = 20; additional.markers[1].id = 21;
  additional.markers[0].header.stamp = ros::Time(1); additional.markers[1].header.stamp = ros::Time(2);
  auv::append_marker_array(additional, &destination);
  ASSERT_EQ(destination.markers.size(), 3U); EXPECT_EQ(destination.markers[1].header.stamp, ros::Time(1));
  visualization_msgs::MarkerArray replaced;
  auv::append_marker_array(additional, &replaced, ros::Time(9));
  ASSERT_EQ(replaced.markers.size(), 2U); EXPECT_EQ(replaced.markers[0].header.stamp, ros::Time(9));
  EXPECT_EQ(replaced.markers[1].header.stamp, ros::Time(9));
}
