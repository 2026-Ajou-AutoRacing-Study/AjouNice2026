#include "autoware/behavior_path_static_obstacle_avoidance_module/candidate_sampling.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp"
#include "autoware/motion_utils/trajectory/drivable_area_footprint.hpp"
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <gtest/gtest.h>

namespace bp = autoware::behavior_path_planner;
namespace area = autoware::motion_utils::drivable_area;

TEST(BoundaryCandidates, FallbackNeverTreatsRoadBorderOrUnknownAsLaneMarking)
{
  EXPECT_FALSE(bp::boundary_sampling::mayCrossBoundary("road_border", ""));
  EXPECT_FALSE(bp::boundary_sampling::mayCrossBoundary("curbstone", "low"));
  EXPECT_FALSE(bp::boundary_sampling::mayCrossBoundary("", ""));
  EXPECT_TRUE(bp::boundary_sampling::mayCrossBoundary("line_thin", "solid"));
  EXPECT_TRUE(bp::boundary_sampling::mayCrossBoundary("line_thin", "dashed"));
  EXPECT_TRUE(bp::boundary_sampling::mayCrossBoundary("virtual", ""));
}

static auto vehicle()
{
  return autoware::vehicle_info_utils::createVehicleInfo(
    0.3, 0.2, 3.0, 1.6, 0.845, 0.79, 0.2, 0.2, 1.5, 0.698);
}

static geometry_msgs::Point point(double x, double y)
{
  geometry_msgs::Point p; p.x = x; p.y = y; return p;
}

static autoware_utils::Polygon2d square(double min_x, double min_y, double size)
{
  autoware_utils::Polygon2d polygon;
  polygon.outer() = {
    {min_x, min_y}, {min_x + size, min_y}, {min_x + size, min_y + size},
    {min_x, min_y + size}, {min_x, min_y}};
  boost::geometry::correct(polygon);
  return polygon;
}

TEST(BoundaryCandidates, RejectsObjectHardMarginViolation)
{
  const auto ego = square(0.0, 0.0, 1.0);
  EXPECT_TRUE(bp::boundary_sampling::hasHardClearance(ego, square(1.2, 0.0, 1.0), 0.1));
  EXPECT_FALSE(bp::boundary_sampling::hasHardClearance(ego, square(1.2, 0.0, 1.0), 0.3));
  EXPECT_FALSE(bp::boundary_sampling::hasHardClearance(ego, square(0.8, 0.0, 1.0), 0.0));
}

TEST(BoundaryCandidates, RotatedFrontCornerIsRejected)
{
  const std::vector<geometry_msgs::Point> left{point(-10, 1.5), point(50, 1.5)};
  const std::vector<geometry_msgs::Point> right{point(-10, -1.5), point(50, -1.5)};
  geometry_msgs::Pose pose; pose.orientation.w = 1.0;
  EXPECT_FALSE(area::isOutsideDrivableAreaFromRectangleFootprint(pose, left, right, vehicle(), false));
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.3);
  EXPECT_TRUE(area::isOutsideDrivableAreaFromRectangleFootprint(pose, left, right, vehicle(), false));
  EXPECT_TRUE(area::isOutsideDrivableAreaFromRectangleFootprint(pose, left, right, vehicle(), true));
}

TEST(BoundaryCandidates, RotatedRearCornerIsRejected)
{
  const std::vector<geometry_msgs::Point> left{point(-10, 1.5), point(50, 1.5)};
  const std::vector<geometry_msgs::Point> right{point(-10, -1.5), point(50, -1.5)};
  geometry_msgs::Pose pose; pose.position.y = 0.5;
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(-0.1);
  EXPECT_TRUE(area::isOutsideDrivableAreaFromRectangleFootprint(pose, left, right, vehicle(), false));
}

TEST(BoundaryCandidates, ReturnIsNotExemptFromBoundaryCheck)
{
  const std::vector<geometry_msgs::Point> left{point(-10, 1.5), point(50, 1.5)};
  const std::vector<geometry_msgs::Point> right{point(-10, -1.5), point(50, -1.5)};
  geometry_msgs::Pose pose; pose.orientation.w = 1.0;
  pose.position.y = 0.7;
  EXPECT_TRUE(area::isOutsideDrivableAreaFromRectangleFootprint(pose, left, right, vehicle(), false));
  pose.position.y = 0.0;
  EXPECT_FALSE(area::isOutsideDrivableAreaFromRectangleFootprint(pose, left, right, vehicle(), false));
}

TEST(BoundaryCandidates, LaterEntryFitsWideningRoad)
{
  const std::vector<geometry_msgs::Point> left{
    point(-10, 1.5), point(10, 1.5), point(15, 5), point(50, 5)};
  const std::vector<geometry_msgs::Point> right{point(-10, -1.5), point(50, -1.5)};
  geometry_msgs::Pose pose; pose.position.x = 5; pose.position.y = 1;
  pose.orientation.w = 1;
  EXPECT_TRUE(area::isOutsideDrivableAreaFromRectangleFootprint(pose, left, right, vehicle(), false));
  pose.position.x = 20;
  EXPECT_FALSE(area::isOutsideDrivableAreaFromRectangleFootprint(pose, left, right, vehicle(), false));
}

class IntervalSampling : public ::testing::Test
{
protected:
  bp::PathWithLaneId path;
  std::vector<double> arc;
  bp::AvoidLineArray lines;
  void SetUp() override
  {
    path.points.resize(61);
    for (size_t i = 0; i < path.points.size(); ++i) {
      arc.push_back(i);
      path.points[i].point.pose.position.x = i;
      path.points[i].point.pose.orientation.w = 1;
    }
    bp::AvoidLine entry;
    entry.start_idx = 5; entry.end_idx = 25;
    entry.start_longitudinal = 5; entry.end_longitudinal = 25;
    entry.start_shift_length = 0; entry.end_shift_length = 2;
    auto back = entry;
    back.start_idx = 30; back.end_idx = 50;
    back.start_longitudinal = 30; back.end_longitudinal = 50;
    back.start_shift_length = 2; back.end_shift_length = 0;
    lines = {entry, back};
  }
};

TEST_F(IntervalSampling, ShortensEntryAndReturnWithoutChangingClearanceTargets)
{
  ASSERT_TRUE(bp::boundary_sampling::shorten(lines, path, arc, 0.5, 0.5));
  EXPECT_EQ(lines[0].start_idx, 15u); EXPECT_EQ(lines[0].end_idx, 25u);
  EXPECT_EQ(lines[1].start_idx, 30u); EXPECT_EQ(lines[1].end_idx, 40u);
  EXPECT_EQ(lines[0].end_shift_length, 2); EXPECT_EQ(lines[1].end_shift_length, 0);
}

TEST_F(IntervalSampling, DoesNotMoveStartedShift)
{
  lines[0].start_longitudinal = -1;
  ASSERT_TRUE(bp::boundary_sampling::shorten(lines, path, arc, 0.35, 0.35));
  EXPECT_EQ(lines[0].start_idx, 5u); EXPECT_EQ(lines[0].end_idx, 25u);
}

TEST_F(IntervalSampling, NominalCandidateUnchanged)
{
  ASSERT_TRUE(bp::boundary_sampling::shorten(lines, path, arc, 1, 1));
  EXPECT_EQ(lines[0].start_idx, 5u); EXPECT_EQ(lines[1].end_idx, 50u);
}

TEST_F(IntervalSampling, RejectsCollapsedIntervalsAndInvalidInputs)
{
  EXPECT_FALSE(bp::boundary_sampling::shorten(lines, path, arc, 0, 1));
  EXPECT_FALSE(bp::boundary_sampling::shorten(lines, path, arc, 0.001, 1));
  arc.pop_back();
  EXPECT_FALSE(bp::boundary_sampling::shorten(lines, path, arc, 1, 1));
}

TEST_F(IntervalSampling, SamplesLateralDepthAndPreservesReturnToCenter)
{
  ASSERT_TRUE(bp::boundary_sampling::scaleLateralExcursion(lines, 0.7));
  EXPECT_DOUBLE_EQ(lines[0].start_shift_length, 0.0);
  EXPECT_DOUBLE_EQ(lines[0].end_shift_length, 1.4);
  EXPECT_DOUBLE_EQ(lines[1].start_shift_length, 1.4);
  EXPECT_DOUBLE_EQ(lines[1].end_shift_length, 0.0);
}

TEST_F(IntervalSampling, DoesNotRescaleActiveOrAlreadyShiftedManeuver)
{
  lines[0].start_longitudinal = 0.1;
  EXPECT_FALSE(bp::boundary_sampling::scaleLateralExcursion(lines, 0.7));

  lines[0].start_longitudinal = 5.0;
  lines[0].start_shift_length = 0.2;
  EXPECT_FALSE(bp::boundary_sampling::scaleLateralExcursion(lines, 0.7));
}

TEST_F(IntervalSampling, RejectsInvalidLateralDepthRatio)
{
  EXPECT_FALSE(bp::boundary_sampling::scaleLateralExcursion(lines, 0.0));
  EXPECT_FALSE(bp::boundary_sampling::scaleLateralExcursion(lines, 1.1));
  EXPECT_TRUE(bp::boundary_sampling::scaleLateralExcursion(lines, 1.0));
}
