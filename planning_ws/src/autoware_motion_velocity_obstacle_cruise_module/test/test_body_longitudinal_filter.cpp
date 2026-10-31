#include "body_longitudinal_filter.hpp"
#include <gtest/gtest.h>
#include <cstdlib>
#include <cmath>
#include <fstream>

namespace
{
using autoware::motion_velocity_planner::body_longitudinal_bounds;
using Point = autoware_planning_msgs::TrajectoryPoint;
geometry_msgs::Pose pose(double x, double y = 0, double yaw = 0)
{
  geometry_msgs::Pose p;
  p.position.x = x; p.position.y = y;
  p.orientation.z = std::sin(yaw / 2); p.orientation.w = std::cos(yaw / 2);
  return p;
}
std::vector<Point> straight()
{
  std::vector<Point> points;
  for (int i = -30; i <= 30; ++i) { Point p; p.pose = pose(i); points.push_back(p); }
  return points;
}
autoware::vehicle_info_utils::VehicleInfo vehicle()
{
  autoware::vehicle_info_utils::VehicleInfo v{};
  v.max_longitudinal_offset_m = 4; v.rear_overhang_m = 1; v.vehicle_width_m = 2;
  return v;
}
autoware_perception_msgs::PredictedObject object(double x, double y, double yaw = 0)
{
  autoware_perception_msgs::PredictedObject o;
  o.kinematics.initial_pose_with_covariance.pose = pose(x, y, yaw);
  o.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  o.shape.dimensions.x = 8.8; o.shape.dimensions.y = 3.2; o.shape.dimensions.z = 3;
  return o;
}
TEST(BodyLongitudinalFilter, RearReferenceDoesNotDiscardAdjacentLongTruck)
{
  const auto r = body_longitudinal_bounds(straight(), pose(0), object(-0.5, 3), vehicle());
  ASSERT_TRUE(r); EXPECT_FALSE(r->fully_behind()); EXPECT_NEAR(r->object_max_s, 3.9, 1e-6);
}
TEST(BodyLongitudinalFilter, ExcludesOnlyAfterTruckFrontClearsEgoRear)
{
  auto o = object(-5.4, 3);
  auto r = body_longitudinal_bounds(straight(), pose(0), o, vehicle());
  ASSERT_TRUE(r); EXPECT_FALSE(r->fully_behind()); // exact bumper boundary
  o.kinematics.initial_pose_with_covariance.pose.position.x = -5.5;
  r = body_longitudinal_bounds(straight(), pose(0), o, vehicle());
  ASSERT_TRUE(r); EXPECT_TRUE(r->fully_behind());
}
TEST(BodyLongitudinalFilter, AccountsForEgoRearOverhang)
{
  auto v = vehicle(); v.rear_overhang_m = 3;
  const auto r = body_longitudinal_bounds(straight(), pose(0), object(-6, 3), v);
  ASSERT_TRUE(r); EXPECT_FALSE(r->fully_behind());
}
TEST(BodyLongitudinalFilter, UsesWidthAndHeadingForCrossingObject)
{
  const auto r = body_longitudinal_bounds(straight(), pose(0), object(-2, 5, M_PI_2), vehicle());
  ASSERT_TRUE(r); EXPECT_NEAR(r->object_max_s, -0.4, 1e-6); EXPECT_FALSE(r->fully_behind());
}
TEST(BodyLongitudinalFilter, UsesEgoHeadingAndWidth)
{
  auto v = vehicle(); v.vehicle_width_m = 6;
  const auto r = body_longitudinal_bounds(straight(), pose(0, 0, M_PI_2), object(-7, 5), v);
  ASSERT_TRUE(r); EXPECT_NEAR(r->ego_min_s, -3, 1e-6); EXPECT_FALSE(r->fully_behind());
}
TEST(BodyLongitudinalFilter, HandlesCurvedTrajectory)
{
  std::vector<Point> path;
  for (int i = 0; i <= 90; ++i) {
    double a = i * M_PI / 180; Point p; p.pose = pose(20*std::sin(a),20*(1-std::cos(a)),a);
    path.push_back(p);
  }
  const auto r = body_longitudinal_bounds(path, pose(20,20,M_PI_2), object(20,19,M_PI_2), vehicle());
  ASSERT_TRUE(r); EXPECT_FALSE(r->fully_behind());
}
TEST(BodyLongitudinalFilter, UsesPolygonFootprintRatherThanBoxDimensions)
{
  auto o = object(-3, 4); o.shape.type = autoware_perception_msgs::Shape::POLYGON;
  for (const auto & xy : std::vector<std::pair<float,float>>{{-1,-1},{5,-1},{5,1},{-1,1}}) {
    geometry_msgs::Point32 p; p.x=xy.first; p.y=xy.second; o.shape.footprint.points.push_back(p);
  }
  const auto r = body_longitudinal_bounds(straight(),pose(0),o,vehicle());
  ASSERT_TRUE(r); EXPECT_NEAR(r->object_max_s,2,1e-6); EXPECT_FALSE(r->fully_behind());
}
TEST(BodyLongitudinalFilter, HandlesCylinder)
{
  auto o=object(-2,3); o.shape.type=autoware_perception_msgs::Shape::CYLINDER;
  o.shape.dimensions.x=4; o.shape.dimensions.y=4;
  const auto r=body_longitudinal_bounds(straight(),pose(0),o,vehicle());
  ASSERT_TRUE(r); EXPECT_FALSE(r->fully_behind());
}
TEST(BodyLongitudinalFilter, InvalidGeometryCannotEstablishClearance)
{
  auto o=object(-20,3); o.shape.dimensions.x=0; o.shape.dimensions.y=0;
  EXPECT_FALSE(body_longitudinal_bounds(straight(),pose(0),o,vehicle()));
  EXPECT_FALSE(body_longitudinal_bounds({},pose(0),object(-20,3),vehicle()));
  o=object(-20,3); o.kinematics.initial_pose_with_covariance.pose.position.x=NAN;
  EXPECT_FALSE(body_longitudinal_bounds(straight(),pose(0),o,vehicle()));
}
TEST(BodyLongitudinalFilter, RetainsPhysicalOverlap)
{
  const auto r=body_longitudinal_bounds(straight(),pose(0),object(-3,0),vehicle());
  ASSERT_TRUE(r); EXPECT_TRUE(r->footprints_overlap); EXPECT_FALSE(r->fully_behind());
}
// Optional offline recorded-input regression; no simulator or ROS master needed.
TEST(BodyLongitudinalFilter, RecordedCollisionTarget)
{
  const auto file=std::getenv("CRUISE_BODY_FILTER_REPLAY");
  if (!file) GTEST_SKIP();
  std::ifstream in(file); ASSERT_TRUE(in.good());
  size_t count; in>>count; ASSERT_GT(count,0U);
  for(size_t n=0;n<count;++n) {
    double t; size_t size; in>>t>>size;
    std::vector<Point> path(size);
    auto read_pose=[&](geometry_msgs::Pose & p) {
      in>>p.position.x>>p.position.y>>p.orientation.x>>p.orientation.y>>p.orientation.z>>p.orientation.w;
    };
    for(auto & p:path) read_pose(p.pose);
    geometry_msgs::Pose ego; read_pose(ego);
    autoware_perception_msgs::PredictedObject o;
    read_pose(o.kinematics.initial_pose_with_covariance.pose);
    o.shape.type=autoware_perception_msgs::Shape::BOUNDING_BOX;
    in>>o.shape.dimensions.x>>o.shape.dimensions.y>>o.shape.dimensions.z;
    auto v=vehicle(); in>>v.max_longitudinal_offset_m>>v.rear_overhang_m>>v.vehicle_width_m;
    ASSERT_FALSE(in.fail());
    const auto r=body_longitudinal_bounds(path,ego,o,v);
    ASSERT_TRUE(r)<<t; EXPECT_FALSE(r->fully_behind())<<t;
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
