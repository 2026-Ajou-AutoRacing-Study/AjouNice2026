#include <gtest/gtest.h>

#include "autoware/objects_of_interest_marker_interface/marker_utils.hpp"
#include "autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp"

namespace aw = autoware::objects_of_interest_marker_interface;

TEST(ObjectsOfInterestMarkerInterface, PreservesMarkerGeometryAndColors)
{
  autoware_perception_msgs::Shape shape;
  shape.dimensions.z = 2.0;
  geometry_msgs::Pose pose;
  pose.position.x = 3.0;
  pose.position.y = -4.0;
  pose.position.z = 1.0;
  const auto data = aw::ObjectMarkerData{pose, shape, aw::ObjectsOfInterestMarkerInterface::getColor(aw::ColorName::GREEN)};
  const auto markers = aw::marker_utils::createTargetMarker(7, data, "avoidance", 0.5);
  ASSERT_EQ(markers.markers.size(), 4U);
  EXPECT_EQ(markers.markers[0].header.frame_id, "map");
  EXPECT_EQ(markers.markers[0].ns, "avoidance_arrow");
  EXPECT_EQ(markers.markers[0].type, visualization_msgs::Marker::ARROW);
  ASSERT_EQ(markers.markers[0].points.size(), 2U);
  EXPECT_DOUBLE_EQ(markers.markers[0].points[0].z, 3.5);
  EXPECT_DOUBLE_EQ(markers.markers[0].points[1].z, 2.5);
  EXPECT_EQ(markers.markers[1].type, visualization_msgs::Marker::LINE_STRIP);
  EXPECT_EQ(markers.markers[1].points.size(), 21U);
  EXPECT_EQ(markers.markers[3].type, visualization_msgs::Marker::TEXT_VIEW_FACING);
  EXPECT_EQ(markers.markers[3].text, "avoidance");
  const auto amber = aw::ObjectsOfInterestMarkerInterface::getColor(aw::ColorName::AMBER, 0.5F);
  EXPECT_NEAR(amber.r, 1.0F, 1e-6F);
  EXPECT_NEAR(amber.g, 234.0F / 255.0F, 1e-6F);
  EXPECT_NEAR(amber.b, 0.0F, 1e-6F);
  EXPECT_FLOAT_EQ(amber.a, 0.5F);
}
