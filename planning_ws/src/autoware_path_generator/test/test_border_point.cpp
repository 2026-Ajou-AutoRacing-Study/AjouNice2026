#include <autoware/path_generator/border_point.hpp>
#include <gtest/gtest.h>
namespace {
lanelet::ConstLanelet make_lanelet() {
  lanelet::LineString3d left(lanelet::InvalId, {lanelet::Point3d(lanelet::InvalId, 0., 1., 0.), lanelet::Point3d(lanelet::InvalId, 4., 1., 0.)});
  lanelet::LineString3d right(lanelet::InvalId, {lanelet::Point3d(lanelet::InvalId, 0., -1., 0.), lanelet::Point3d(lanelet::InvalId, 4., -1., 0.)});
  return lanelet::Lanelet(lanelet::InvalId, left, right);
}
}
// Original get_border_point intersects 3D segments. The old reduced adapter
// projected to 2D and accepted a vertically separated crossing; retain the
// fixture as a rejection regression rather than preserving that wrong oracle.
TEST(PathGeneratorBorderPoint, RejectsHeightSeparatedCrossing) {
  const lanelet::BasicLineString3d segment{{-1., 0., 2.}, {1., 0., 7.}};
  const auto point = autoware::path_generator::utils::get_border_point(segment, make_lanelet());
  EXPECT_FALSE(point);
}
TEST(PathGeneratorBorderPoint, ReturnsCoplanarCrossing) {
  const lanelet::BasicLineString3d segment{{-1., 0., 0.}, {1., 0., 0.}};
  const auto point = autoware::path_generator::utils::get_border_point(segment, make_lanelet());
  ASSERT_TRUE(point); EXPECT_DOUBLE_EQ(point->x(), 0.); EXPECT_DOUBLE_EQ(point->y(), 0.); EXPECT_DOUBLE_EQ(point->z(), 0.);
}
TEST(PathGeneratorBorderPoint, FailsClosedWithoutCrossingOrSegment) {
  const auto lanelet = make_lanelet();
  EXPECT_FALSE(autoware::path_generator::utils::get_border_point({{1., 0., 0.}}, lanelet));
  EXPECT_FALSE(autoware::path_generator::utils::get_border_point({{1., 2., 0.}, {2., 2., 0.}}, lanelet));
}
TEST(PathGeneratorLaneletContainment, AcceptsAnyContainingLaneletAndRejectsOutside) {
  geometry_msgs::Pose inside;
  inside.position.x = 2.; inside.position.y = 0.;
  geometry_msgs::Pose outside;
  outside.position.x = 2.; outside.position.y = 3.;
  const lanelet::ConstLanelets lanelets{make_lanelet()};
  EXPECT_TRUE(autoware::path_generator::utils::is_pose_inside_lanelets(inside, lanelets));
  EXPECT_FALSE(autoware::path_generator::utils::is_pose_inside_lanelets(outside, lanelets));
  EXPECT_FALSE(autoware::path_generator::utils::is_pose_inside_lanelets(inside, {}));
}
int main(int argc,char ** argv) { ::testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
