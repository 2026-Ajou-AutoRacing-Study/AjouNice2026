// Copyright 2026 AjouNice2026
// Licensed under the Apache License, Version 2.0.
#include <autoware/path_optimizer/node.hpp>
#include <autoware/path_optimizer/obstacle_collision.hpp>
#include <gtest/gtest.h>

namespace autoware::path_optimizer
{
namespace
{
PlannerData scene(bool touching)
{
  PlannerData d; d.header.frame_id="map"; d.header.stamp=ros::Time::now();
  d.ego_pose.position.x=5.0; d.ego_pose.orientation.w=1.0; d.ego_vel=1.0;
  d.optimization_forward_length=50.0;
  for (int i=0;i<=400;++i) {
    TrajectoryPoint p; p.pose.orientation.w=1.0; p.pose.position.x=i*0.2;
    p.longitudinal_velocity_mps=5.0; d.traj_points.push_back(p);
    p.pose.position.y=5.0; d.left_bound.push_back(p.pose.position);
    p.pose.position.y=-5.0; d.right_bound.push_back(p.pose.position);
  }
  autoware_planning_msgs::StaticObstacle o;
  const double x=touching?5.0:16.0, y=touching?1.0:2.0;
  for (const auto & xy:std::vector<std::pair<double,double>>{{x,y},{x+2,y},{x+2,y+1},{x,y+1}}) {
    geometry_msgs::Point32 p; p.x=xy.first; p.y=xy.second; o.polygon.points.push_back(p);
  }
  o.hard_margin=0.1; o.soft_margin=0.55; d.obstacles.push_back(o); return d;
}
PlannerData narrowScene()
{
  auto d=scene(false);
  for(auto & p:d.left_bound)p.y=1.55;
  for(auto & p:d.right_bound)p.y=-1.55;
  d.obstacles.front().polygon.points.clear();
  for(const auto & xy:std::vector<std::pair<double,double>>{
      {22.0,-0.4},{24.0,-0.4},{24.0,0.4},{22.0,0.4}}) {
    geometry_msgs::Point32 p;p.x=xy.first;p.y=xy.second;
    d.obstacles.front().polygon.points.push_back(p);
  }
  return d;
}
class Harness:public PathOptimizer
{
public:
  Harness():PathOptimizer(ros::NodeHandle(),ros::NodeHandle("~"))
  {
    obstacle_search_time_budget_ms_=1000.0;
  }
  auto run(PlannerData d) {d.allow_initial_obstacle_overlap=enable_obstacle_fallback_;debug_data_ptr_->ego_pose=d.ego_pose;return optimizeTrajectory(d);}
  bool recovering() const {return using_recovery_geometry_;}
  bool failed() const {return is_optimization_failed_;}
  void disable() {enable_obstacle_fallback_=false;}
  bool blockedByFinalCheck(const PlannerData & d,std::vector<TrajectoryPoint> & p) {return stopBeforeObstacle(d,p);}
  bool hitsObstacle(const PlannerData & d,const std::vector<TrajectoryPoint> & p) {
    const size_t ego=autoware::motion_utils::findNearestIndex(p,d.ego_pose.position);
    return firstObstacleCollision(p,ego,d.obstacles,vehicle_info_).has_value();
  }

};
bool moving(const std::vector<TrajectoryPoint> & p) {
  return std::any_of(p.begin(),p.end(),[](const auto & q){return q.longitudinal_velocity_mps>0.1;});
}
}
TEST(RecoveryNode, OrdinarySolutionDoesNotActivateRecovery)
{
  Harness node;auto p=node.run(scene(false));EXPECT_TRUE(moving(p));EXPECT_FALSE(node.recovering());
}
TEST(RecoveryNode, MeasuredMarginOverlapEscapesAndSurvivesFinalCheck)
{
  Harness node;auto d=scene(true);d.allow_initial_obstacle_overlap=true;auto p=node.run(d);
  EXPECT_TRUE(moving(p));EXPECT_FALSE(node.failed());
  EXPECT_FALSE(node.blockedByFinalCheck(d,p));EXPECT_TRUE(moving(p));
  const auto ego = autoware::motion_utils::findNearestIndex(p,d.ego_pose.position);
  EXPECT_DOUBLE_EQ(p[ego].longitudinal_velocity_mps,5.0);
}
TEST(RecoveryNode, DisableRestoresOriginalOverlapStop)
{
  Harness node;node.disable();auto p=node.run(scene(true));
  EXPECT_FALSE(moving(p));EXPECT_FALSE(node.recovering());EXPECT_TRUE(node.failed());
}
TEST(RecoveryNode, UpstreamStopIsNotReleasedByContactRecovery)
{
  Harness node;auto d=scene(true);for (auto & p:d.traj_points)p.longitudinal_velocity_mps=0.0;
  EXPECT_FALSE(moving(node.run(d)));EXPECT_FALSE(node.recovering());
}
TEST(RecoveryNode, FailedHardRoadSolveRetriesWithSoftRoadAndHardObstacle)
{
  auto d=narrowScene();
  Harness strict;strict.disable();
  EXPECT_FALSE(moving(strict.run(d)));
  Harness retry;auto p=retry.run(d);
  ASSERT_FALSE(p.empty());
  EXPECT_TRUE(moving(p));EXPECT_FALSE(retry.failed());EXPECT_TRUE(retry.recovering());
  EXPECT_FALSE(retry.hitsObstacle(d,p));
  EXPECT_FALSE(retry.blockedByFinalCheck(d,p));
  const size_t ego=autoware::motion_utils::findNearestIndex(p,d.ego_pose.position);
  EXPECT_NEAR(p[ego].pose.position.y,0.0,0.05);
  // The only available escape crosses the narrow planning corridor.
  EXPECT_TRUE(std::any_of(p.begin(),p.end(),[](const auto & q) {
    return std::abs(q.pose.position.y)>1.55;
  }));
}
TEST(RecoveryNode, RoadRetryDoesNotReleaseUpstreamStop)
{
  Harness node;auto d=narrowScene();
  for(auto & p:d.traj_points)p.longitudinal_velocity_mps=0.0;
  EXPECT_FALSE(moving(node.run(d)));EXPECT_FALSE(node.recovering());
}
TEST(RecoveryNode, RoadRetryDoesNotReleaseObstacleConstraints)
{
  Harness node;auto d=narrowScene();
  for(auto & p:d.obstacles.front().polygon.points)p.y=p.y<0?-20.0:20.0;
  EXPECT_FALSE(moving(node.run(d)));EXPECT_FALSE(node.recovering());
}

}
int main(int argc,char ** argv)
{
  ros::init(argc,argv,"test_recovery_node");testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
