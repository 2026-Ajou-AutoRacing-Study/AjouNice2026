// Copyright 2026 AjouNice2026
// Licensed under the Apache License, Version 2.0.
#include <autoware/path_optimizer/common_structs.hpp>
#include <autoware/path_optimizer/obstacle_collision.hpp>
#include <autoware/motion_utils/trajectory/static_obstacle_escape.hpp>
#include <gtest/gtest.h>

namespace autoware::path_optimizer
{
namespace
{
auto vehicle()
{
  autoware::vehicle_info_utils::VehicleInfo v;
  v.wheel_base_m=3.0; v.front_overhang_m=0.845; v.rear_overhang_m=0.79;
  v.vehicle_width_m=1.89;
  return v;
}
TrajectoryPoint point(double x, double y=0.0)
{
  TrajectoryPoint p; p.pose.position.x=x; p.pose.position.y=y;
  p.pose.orientation.w=1.0; p.longitudinal_velocity_mps=5.0; return p;
}
auto obstacle(double x, double y, double length=2.0, double width=1.0)
{
  autoware_planning_msgs::StaticObstacle o;
  for (const auto & xy : std::vector<std::pair<double,double>>{
      {x,y},{x+length,y},{x+length,y+width},{x,y+width}}) {
    geometry_msgs::Point32 q; q.x=xy.first; q.y=xy.second; o.polygon.points.push_back(q);
  }
  o.hard_margin=0.1; o.soft_margin=0.7; return o;
}
PlannerData scene(double half_width=4.0)
{
  PlannerData d; d.ego_pose=point(5).pose; d.ego_vel=1.0;
  for (int i=0;i<=200;++i) {
    d.traj_points.push_back(point(i*0.2));
    d.left_bound.push_back(point(i*0.2,half_width).pose.position);
    d.right_bound.push_back(point(i*0.2,-half_width).pose.position);
  }
  return d;
}
std::vector<TrajectoryPoint> straight()
{
  std::vector<TrajectoryPoint> p;
  for (int i=0;i<=125;++i) p.push_back(point(5+i*0.2));
  return p;
}
}
TEST(Recovery, ExistingPhysicalContactCanExit)
{
  auto d=scene(); d.obstacles.push_back(obstacle(5,0.90));
  EXPECT_TRUE(autoware::motion_utils::checkStaticObstacleEscape(
    straight(),d.ego_pose,d.obstacles,vehicle()).valid);
}
TEST(Recovery, ContactIsRankedByCostWithoutAFixedAreaAllowance)
{
  auto d=scene();d.obstacles.push_back(obstacle(5,0.90));
  auto direct=straight(),detour=straight();
  // Both eventually leave the same initially touching object. More scraping
  // remains a higher cost, rather than being cut off at an invented area limit.
  for (size_t i=1;i<detour.size();++i) detour[i].pose.position.y=0.5;
  const auto a=autoware::motion_utils::checkStaticObstacleEscape(direct,d.ego_pose,d.obstacles,vehicle());
  const auto b=autoware::motion_utils::checkStaticObstacleEscape(detour,d.ego_pose,d.obstacles,vehicle());
  ASSERT_TRUE(a.valid);ASSERT_TRUE(b.valid);EXPECT_LT(a.contact_cost,b.contact_cost);
}
TEST(Recovery, AnotherObstacleStillBlocksEscape)
{
  auto d=scene(); d.obstacles={obstacle(5,1.0),obstacle(16,-0.5)};
  EXPECT_FALSE(autoware::motion_utils::checkStaticObstacleEscape(
    straight(),d.ego_pose,d.obstacles,vehicle()).valid);
}
TEST(Recovery, UnclearedPrefixIsRejected)
{
  auto d=scene(); d.obstacles.push_back(obstacle(5,0.90,30));
  EXPECT_FALSE(autoware::motion_utils::checkStaticObstacleEscape(
    straight(),d.ego_pose,d.obstacles,vehicle()).valid);

}
TEST(Recovery, MalformedObstacleIsRejected)
{
  auto d=scene(); d.obstacles.push_back(obstacle(5,1.0));
  d.obstacles[0].hard_margin=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(autoware::motion_utils::checkStaticObstacleEscape(
    straight(),d.ego_pose,d.obstacles,vehicle()).valid);

}
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
