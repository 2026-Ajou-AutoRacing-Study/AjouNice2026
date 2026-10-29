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
  auto run(PlannerData d) {d.allow_initial_obstacle_overlap=false;debug_data_ptr_->ego_pose=d.ego_pose;
    recovery_permitted_=recovery_state_.active() && enable_obstacle_fallback_ &&
      d.traj_points[25].longitudinal_velocity_mps>0.1;return optimizeTrajectory(d);}
  void activate() {
    recovery_permitted_=true;recovery_input_max_age_=10.0;
    auto objects=boost::make_shared<autoware_perception_msgs::PredictedObjects>();
    objects->header.frame_id="map";objects->header.stamp=ros::Time::now();recovery_objects_=objects;
    for(int i=0;i<=22;++i) {
      recovery_state_.observe(i*0.1,0.0,true,true);
      recovery_state_.normalResult(i*0.1,false,true,true);
    }
    ASSERT_TRUE(recovery_state_.active());
  }
  void pathCycle(PlannerData d, const ros::Publisher & odometry, bool behavior_stop=false, bool drive=true) {
    const auto stamp=ros::Time::now();
    autoware_planning_msgs::Path path;
    path.header=d.header;path.header.stamp=stamp;
    path.left_bound=d.left_bound;path.right_bound=d.right_bound;
    for(const auto & point:d.traj_points) {
      autoware_planning_msgs::PathPoint p;
      p.pose=point.pose;p.longitudinal_velocity_mps=point.longitudinal_velocity_mps;
      path.points.push_back(p);
    }
    autoware_planning_msgs::StaticObstacleScene scene;
    scene.header=path.header;scene.obstacles=d.obstacles;
    obstacle_scenes_[stamp.toNSec()]=scene;
    auto objects=boost::make_shared<autoware_perception_msgs::PredictedObjects>();
    objects->header=path.header;recovery_objects_=objects;
    autonomous_=true;operation_mode_received_=stamp;external_control_=true;control_mode_received_=stamp;external_stop_=false;external_stop_received_=stamp;
    drive_gear_=drive;gear_received_=stamp;
    behavior_stop_=behavior_stop;behavior_stop_received_=stamp;
    nav_msgs::Odometry odom;odom.header=path.header;odom.pose.pose=d.ego_pose;
    odom.twist.twist.linear.x=d.ego_vel;odometry.publish(odom);
    ros::WallDuration(0.02).sleep();ros::spinOnce();
    onPath(boost::make_shared<autoware_planning_msgs::Path>(path));
    ros::spinOnce();
  }
  void setSearchBudget(double value) {obstacle_search_time_budget_ms_=value;}
  bool active() const {return recovery_state_.active();}
  void resetSeeds() {resetPreviousData();}
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
TEST(RecoveryNode, MeasuredMarginOverlapDoesNotGrantCollisionPermission)
{
  Harness node;auto d=scene(true);d.allow_initial_obstacle_overlap=true;auto p=node.run(d);
  EXPECT_FALSE(moving(p));EXPECT_TRUE(node.failed());EXPECT_FALSE(node.recovering());
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
TEST(RecoveryNode, MovingOrTransientFailureNeverRelaxesRoad)
{
  Harness node; auto d=narrowScene();
  EXPECT_FALSE(moving(node.run(d))); EXPECT_FALSE(node.recovering());
  EXPECT_FALSE(node.active());
}
TEST(RecoveryNode, BoundaryOnlyFailureCanRecoverWithoutStaticObstacles)
{
  Harness node; node.activate(); node.setSearchBudget(0.0); auto d=scene(false); d.obstacles.clear();d.ego_vel=0;
  for(auto & p:d.left_bound)p.y=0.8;
  for(auto & p:d.right_bound)p.y=-0.8;
  auto p=node.run(d);
  EXPECT_TRUE(moving(p));EXPECT_TRUE(node.recovering());EXPECT_TRUE(node.active());
  node.resetSeeds();EXPECT_TRUE(node.active());
  d.ego_vel=2.0; p=node.run(d);
  EXPECT_TRUE(moving(p));EXPECT_TRUE(node.recovering());EXPECT_TRUE(node.active());
}
TEST(RecoveryNode, ActualPathCallbackGatesAndImmediatelyHandsOverNormalOutput)
{
  Harness node;ros::NodeHandle private_node("~");
  auto odometry=private_node.advertise<nav_msgs::Odometry>("input/odometry",1,true);
  autoware_planning_msgs::Trajectory::ConstPtr output;
  auto subscriber=private_node.subscribe<autoware_planning_msgs::Trajectory>("output/path",1,
    [&](const autoware_planning_msgs::Trajectory::ConstPtr & p){output=p;});
  const auto wait_started=ros::WallTime::now();
  while(odometry.getNumSubscribers()==0 && (ros::WallTime::now()-wait_started).toSec()<3) {
    ros::spinOnce();ros::WallDuration(0.02).sleep();
  }
  auto d=scene(false);d.obstacles.clear();d.ego_vel=0;
  for(auto & p:d.left_bound)p.y=0.8;
  for(auto & p:d.right_bound)p.y=-0.8;
  for(int i=0;i<23;++i) {
    node.pathCycle(d,odometry,false,false);ros::WallDuration(0.08).sleep();
  }
  EXPECT_FALSE(node.active()); // Parked preparation is not autonomous driving.
  for(int i=0;i<23;++i) {
    node.pathCycle(d,odometry,true);ros::WallDuration(0.08).sleep();
  }
  EXPECT_FALSE(node.active()); // A behavior stop cannot become recovery authorization.
  for(int i=0;i<30 && !node.recovering();++i) {
    node.pathCycle(d,odometry);ros::WallDuration(0.08).sleep();
  }
  ASSERT_TRUE(node.active());ASSERT_TRUE(node.recovering());
  ros::WallDuration(0.03).sleep();ros::spinOnce();
  ASSERT_TRUE(output);EXPECT_TRUE(output->boundary_recovery);
  EXPECT_TRUE(moving(output->points));
  for(auto & p:d.left_bound)p.y=5;
  for(auto & p:d.right_bound)p.y=-5;
  node.pathCycle(d,odometry);
  EXPECT_FALSE(node.active());EXPECT_FALSE(node.recovering());
  ros::WallDuration(0.03).sleep();ros::spinOnce();
  ASSERT_TRUE(output);EXPECT_FALSE(output->boundary_recovery);EXPECT_TRUE(moving(output->points));
}
TEST(RecoveryNode, FailedHardRoadSolveRetriesWithSoftRoadAndHardObstacle)
{
  auto d=narrowScene();
  Harness strict;strict.disable();
  EXPECT_FALSE(moving(strict.run(d)));
  Harness retry;retry.activate();auto p=retry.run(d);
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
  Harness node;node.activate();auto d=narrowScene();
  for(auto & p:d.traj_points)p.longitudinal_velocity_mps=0.0;
  EXPECT_FALSE(moving(node.run(d)));EXPECT_FALSE(node.recovering());
}
TEST(RecoveryNode, RoadRetryDoesNotReleaseObstacleConstraints)
{
  Harness node;node.activate();auto d=narrowScene();
  for(auto & p:d.obstacles.front().polygon.points)p.y=p.y<0?-20.0:20.0;
  EXPECT_FALSE(moving(node.run(d)));EXPECT_FALSE(node.recovering());
}

}
int main(int argc,char ** argv)
{
  ros::init(argc,argv,"test_recovery_node");testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
