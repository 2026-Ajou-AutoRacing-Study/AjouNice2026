#include <autoware/motion_velocity_planner_common/polygon_utils.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <gtest/gtest.h>

namespace
{
autoware::vehicle_info_utils::VehicleInfo vehicle()
{
  autoware::vehicle_info_utils::VehicleInfo value;
  value.vehicle_width_m = 2.0; value.max_longitudinal_offset_m = 3.0;
  value.rear_overhang_m = 1.0; value.wheel_base_m = 2.5;
  return value;
}
std::vector<autoware_planning_msgs::TrajectoryPoint> trajectory()
{
  std::vector<autoware_planning_msgs::TrajectoryPoint> points(3);
  for (size_t i = 0; i < points.size(); ++i) {
    points.at(i).pose.position.x = 5.0 * i; points.at(i).pose.orientation.w = 1.0;
    points.at(i).longitudinal_velocity_mps = 5.0F;
  }
  return points;
}
}

TEST(MotionVelocityPolygonUtils, CreatesPerPointSweptFootprints)
{
  geometry_msgs::Pose ego; ego.orientation.w = 1.0;
  const auto polygons = autoware::motion_velocity_planner::polygon_utils::createOneStepPolygons(
    trajectory(), vehicle(), ego, 0.0, false, 0.0);
  ASSERT_EQ(polygons.size(), 3U);
  EXPECT_TRUE(boost::geometry::covered_by(autoware_utils_geometry::Point2d{3.0, 0.0}, polygons.at(0)));
  EXPECT_TRUE(boost::geometry::covered_by(autoware_utils_geometry::Point2d{7.0, 0.0}, polygons.at(1)));
  EXPECT_TRUE(boost::geometry::covered_by(autoware_utils_geometry::Point2d{3.0, 0.0}, polygons.at(1)));
}

TEST(MotionVelocityPolygonUtils, AppliesLateralMarginToFootprint)
{
  geometry_msgs::Pose ego; ego.orientation.w = 1.0;
  const auto polygons = autoware::motion_velocity_planner::polygon_utils::createOneStepPolygons(
    trajectory(), vehicle(), ego, 0.5, false, 0.0);
  ASSERT_EQ(polygons.size(), 3U);
  // The upstream first-point special case deliberately leaves the rear half unexpanded.
  EXPECT_TRUE(boost::geometry::covered_by(autoware_utils_geometry::Point2d{2.7, 1.4}, polygons.at(0)));
  EXPECT_FALSE(boost::geometry::covered_by(autoware_utils_geometry::Point2d{2.7, 1.6}, polygons.at(0)));
}

TEST(MotionVelocityPolygonUtils, ComputesSourceOffTrackingOnCurvedPath)
{
  auto curved = trajectory();
  curved.at(1).pose.position.y = 1.0; curved.at(1).pose.orientation.z = 0.1;
  curved.at(2).pose.position.y = 3.0; curved.at(2).pose.orientation.z = 0.2;
  const auto offsets = autoware::motion_velocity_planner::polygon_utils::calcFrontOuterWheelOffTracking(curved, vehicle());
  ASSERT_EQ(offsets.size(), curved.size());
  EXPECT_NE(offsets.at(1), 0.0);
}

TEST(MotionVelocityPolygonUtils, ConvergesPoseErrorWithoutMutatingReferenceTrajectory)
{
  const auto points = trajectory();
  geometry_msgs::Pose ego;
  ego.position.y = 1.0;
  ego.orientation.w = 1.0;
  const auto projected = autoware::motion_velocity_planner::polygon_utils::calculateErrorPoses(
    points, ego, 1.0);
  ASSERT_FALSE(projected.empty());
  EXPECT_GT(projected.front().position.y, points.front().pose.position.y);
  EXPECT_LE(projected.size(), points.size());
  EXPECT_DOUBLE_EQ(points.front().pose.position.y, 0.0);
}

TEST(MotionVelocityPolygonUtils, SelectsNearestPolygonCollisionUsingBumperDistance)
{
  const auto points = trajectory();
  std::vector<autoware_utils_geometry::Polygon2d> polygons(3);
  for (size_t i = 0; i < polygons.size(); ++i) {
    const double x = 5.0 * i;
    polygons.at(i).outer() = {{x - 1.0, -1.0}, {x + 3.0, -1.0}, {x + 3.0, 1.0}, {x - 1.0, 1.0}, {x - 1.0, -1.0}};
    boost::geometry::correct(polygons.at(i));
  }
  autoware_utils_geometry::Polygon2d obstacle;
  obstacle.outer() = {{9.0, -0.5}, {11.0, -0.5}, {11.0, 0.5}, {9.0, 0.5}, {9.0, -0.5}};
  geometry_msgs::Point position; position.x = 10.0;
  const auto collision = autoware::motion_velocity_planner::polygon_utils::getCollisionPoint(
    points, polygons, position, obstacle, 3.0);
  ASSERT_TRUE(collision);
  EXPECT_LT(collision->second, 10.0);
  EXPECT_GT(collision->second, 0.0);
}

TEST(MotionVelocityPolygonUtils, OriginalDecimationArgumentIsNotOffTrackingScale)
{
  auto points=trajectory();
  points.at(1).pose.position.y=1.0;
  points.at(2).pose.position.y=3.0;
  geometry_msgs::Pose ego; ego.orientation.w=1.0;
  namespace polygon=autoware::motion_velocity_planner::polygon_utils;
  const auto native=polygon::createOneStepPolygons(points,vehicle(),ego,0.0,false,0.0,0.7);
  const auto original=polygon::create_one_step_polygons(points,vehicle(),ego,0.0,false,0.0,99.0,0.7);
  ASSERT_EQ(native.size(),original.size());
  for(size_t i=0;i<native.size();++i) {
    ASSERT_EQ(native[i].outer().size(),original[i].outer().size());
    for(size_t j=0;j<native[i].outer().size();++j) {
      EXPECT_DOUBLE_EQ(native[i].outer()[j].x(),original[i].outer()[j].x());
      EXPECT_DOUBLE_EQ(native[i].outer()[j].y(),original[i].outer()[j].y());
    }
  }
}

int main(int argc, char ** argv) { ::testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
