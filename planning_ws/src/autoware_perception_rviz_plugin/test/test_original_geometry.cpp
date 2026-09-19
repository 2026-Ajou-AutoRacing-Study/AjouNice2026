#include <autoware_perception_rviz_plugin/object_detection/object_polygon_detail.hpp>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <rviz/display.h>

namespace detail = autoware::rviz_plugins::object_detection::detail;

TEST(OriginalGeometry, BoundingBoxSkeletonAndFill)
{
  autoware_perception_msgs::Shape shape;
  shape.type = shape.BOUNDING_BOX;
  shape.dimensions.x = 4; shape.dimensions.y = 2; shape.dimensions.z = 1.5;
  geometry_msgs::Point center; center.x = 5;
  geometry_msgs::Quaternion orientation; orientation.w = 1;
  std_msgs::ColorRGBA color; color.r = 1; color.a = 0.8;
  const auto wire = detail::get_shape_marker_ptr(shape, center, orientation, color, 0.1,
    true, detail::ObjectFillType::Skeleton);
  EXPECT_EQ(wire->type, visualization_msgs::Marker::LINE_LIST);
  EXPECT_GE(wire->points.size(), 24u);
  EXPECT_EQ(wire->points.size() % 2, 0u);
  EXPECT_DOUBLE_EQ(wire->pose.position.x, 5);
  EXPECT_DOUBLE_EQ(wire->lifetime.toSec(), 0.15);
  const auto filled = detail::get_shape_marker_ptr(shape, center, orientation, color, 0.1,
    true, detail::ObjectFillType::Fill);
  EXPECT_EQ(filled->type, visualization_msgs::Marker::CUBE);
  EXPECT_DOUBLE_EQ(filled->scale.x, 4);
  EXPECT_DOUBLE_EQ(filled->scale.y, 2);
  EXPECT_DOUBLE_EQ(filled->scale.z, 1.5);
}

TEST(OriginalGeometry, VelocityUsesThreeDimensionalMagnitude)
{
  geometry_msgs::Twist twist; twist.linear.x = 3; twist.linear.y = 4;
  const auto marker = detail::get_velocity_text_marker_ptr(
    twist, geometry_msgs::Point{}, std_msgs::ColorRGBA{});
  EXPECT_EQ(marker->text, "18[km/h]");
  EXPECT_EQ(marker->type, visualization_msgs::Marker::TEXT_VIEW_FACING);
  EXPECT_DOUBLE_EQ(marker->lifetime.toSec(), 0.15);
}

TEST(OriginalGeometry, EmptyClassificationFallsBackToUnknown)
{
  std::vector<autoware_perception_msgs::ObjectClassification> labels;
  EXPECT_EQ(detail::get_best_label(labels, "aw149_geometry_test"),
    autoware_perception_msgs::ObjectClassification::UNKNOWN);
}

TEST(OriginalPluginLibrary, AllFourClassesAreDeclaredAndLoadable)
{
  pluginlib::ClassLoader<rviz::Display> loader("rviz", "rviz::Display");
  for (const auto * name : {"DetectedObjects", "TrackedObjects", "PredictedObjects", "TrafficLight"}) {
    const std::string id = std::string("autoware_perception_rviz_plugin/") + name;
    ASSERT_TRUE(loader.isClassAvailable(id)) << id;
    EXPECT_NO_THROW(loader.loadLibraryForClass(id));
    EXPECT_TRUE(loader.isClassLoaded(id));
  }
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "aw149_original_perception_geometry_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
