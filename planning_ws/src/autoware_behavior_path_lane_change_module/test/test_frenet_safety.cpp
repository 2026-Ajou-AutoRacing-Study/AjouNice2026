#include "autoware/behavior_path_lane_change_module/utils/frenet_safety.hpp"
#include "autoware/behavior_path_lane_change_module/utils/current_lane_cancel.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"
#include <gtest/gtest.h>
namespace bpp=autoware::behavior_path_planner;
namespace lc=bpp::utils::lane_change;
namespace {
lc::SafetyPath path(){
  lc::SafetyPath p;
  for(int i=0;i<=150;++i){bpp::PathPointWithLaneId q;q.point.pose.position.x=i;q.point.pose.orientation.w=1;q.point.longitudinal_velocity_mps=10;q.lane_ids={42};p.points.push_back(q);}
  return p;
}
geometry_msgs::Pose pose(double x){geometry_msgs::Pose p;p.position.x=x;p.orientation.w=1;return p;}
struct Fixture {
  std::shared_ptr<bpp::lane_change::CommonData> data=std::make_shared<bpp::lane_change::CommonData>();
  bpp::lane_change::TrajectoryGroup group;
  Fixture(double prepare){
    data->lc_param_ptr=std::make_shared<bpp::lane_change::Parameters>();
    data->bpp_param_ptr=std::make_shared<bpp::BehaviorPathPlannerParameters>();
    data->lc_param_ptr->trajectory.min_prepare_duration=0;
    data->lc_param_ptr->safety.collision_check.prediction_time_resolution=.5;
    data->bpp_param_ptr->ego_nearest_dist_threshold=3;
    data->bpp_param_ptr->ego_nearest_yaw_threshold=1;
    auto odom=boost::make_shared<nav_msgs::Odometry>();odom->pose.pose=pose(0);odom->twist.twist.linear.x=10;data->self_odometry_ptr=odom;
    group.prepare=path();group.prepare_metric.duration=prepare;group.prepare_metric.velocity=10;
    group.prepare_metric.actual_lon_accel=0;group.prepare_metric.sampled_lon_accel=0;
    for(int i=0;i<=10;++i){group.lane_changing.times.push_back(i*.5);group.lane_changing.poses.push_back(pose(prepare*10+i*5));group.lane_changing.longitudinal_velocities.push_back(10);}
  }
  lc::SafetyPrediction native(){return lc::restore_frenet_transition(lc::convert_to_predicted_path(data,group,1),group);}
};
}
TEST(FrenetSafety, RealConverterPrepareZeroRestoresMissingTimeZero){
  Fixture f(0);
  const auto original=lc::convert_to_predicted_path(f.data,f.group,1);
  ASSERT_GT(original.front().time,0);
  const auto native=f.native();EXPECT_DOUBLE_EQ(native.front().time,0);
  const auto p=lc::frenet_observation_hypothesis(path(),native,pose(0),10,3,.5,2,4);
  EXPECT_DOUBLE_EQ(p.front().time,0);EXPECT_NEAR(p.front().pose.position.x,0,1e-6);
  EXPECT_DOUBLE_EQ(p.back().time,3);EXPECT_NEAR(p.back().pose.position.x,30,1e-6);
}
TEST(FrenetSafety, PositivePrepareRestoresExactNativeTransition){
  Fixture f(2);const auto n=f.native();
  const auto it=std::find_if(n.begin(),n.end(),[](auto&p){return p.time==2;});
  ASSERT_NE(it,n.end());EXPECT_DOUBLE_EQ(it->pose.position.x,20);
  const auto p=lc::frenet_observation_hypothesis(path(),n,pose(5),10,3,.5,2,4);
  EXPECT_NEAR(p.front().pose.position.x,5,1e-6);EXPECT_NEAR(p.back().pose.position.x,35,1e-6);
}
TEST(FrenetSafety, NewerObjectTimeAdvancesNativeAndTailWithoutInventingCoverage){
  Fixture f(2);
  for(double x:{0.,25.,68.,75.}){
    const auto model=lc::frenet_observation_hypothesis(path(),f.native(),pose(x),10,3.01,.5,2,4);
    const auto aligned=lc::rebase_observation_prediction(model,.01,3);
    EXPECT_NEAR(aligned.front().pose.position.x,x+.1,1e-6);
    EXPECT_DOUBLE_EQ(aligned.back().time,3);
    EXPECT_NEAR(aligned.back().pose.position.x,x+30.1,1e-6);
  }
  auto short_path=path();short_path.points.resize(100);
  EXPECT_THROW(lc::frenet_observation_hypothesis(short_path,f.native(),pose(75),10,3.01,.5,2,4),std::logic_error);
}
TEST(FrenetSafety, RunningInsideLcRebasesNativeTimeInsteadOfRestartingPrepare){
  Fixture f(2);const auto p=lc::frenet_observation_hypothesis(path(),f.native(),pose(25),10,3,.5,2,4);
  EXPECT_DOUBLE_EQ(p.front().time,0);EXPECT_NEAR(p.front().pose.position.x,25,1e-6);
  EXPECT_NEAR(p.back().pose.position.x,55,1e-6);
}
TEST(FrenetSafety, ConverterCalledWithAdvancedEgoTrimsUnusedPreparePrefix){
  Fixture f(2);
  f.group.prepare.points.resize(21);  // Real prepare segment ends at LC start.
  auto odom=boost::make_shared<nav_msgs::Odometry>();odom->pose.pose=pose(25);odom->twist.twist.linear.x=10;
  f.data->self_odometry_ptr=odom;
  const auto native=f.native();
  const auto p=lc::frenet_observation_hypothesis(path(),native,pose(25),10,3,.5,2,4);
  EXPECT_NEAR(p.front().pose.position.x,25,1e-6);
  EXPECT_NEAR(p.back().pose.position.x,55,1e-6);
}
TEST(FrenetSafety, NearEndAndCompletedLcUseReachableTailNot150KphCeiling){
  Fixture f(2);auto final=path();for(size_t i=71;i<final.points.size();++i)final.points[i].point.longitudinal_velocity_mps=41.667;
  const auto p=lc::frenet_observation_hypothesis(final,f.native(),pose(68),10,3,.5,2,4);
  EXPECT_NEAR(p.back().pose.position.x,105.84,1e-3);EXPECT_NEAR(p.back().velocity,15.6,1e-3);
  const auto done=lc::frenet_observation_hypothesis(final,f.native(),pose(75),10,3,.5,2,4);
  EXPECT_NEAR(done.front().pose.position.x,75,1e-6);EXPECT_NEAR(done.back().pose.position.x,114,1e-3);
}
TEST(FrenetSafety, NarrowFinalCapOrStopCannotHideBetweenRegularSamples){
  Fixture f(2);auto final=path();final.points[28].point.longitudinal_velocity_mps=5;
  EXPECT_THROW(lc::frenet_observation_hypothesis(final,f.native(),pose(25),10,3,.5,2,4),std::logic_error);
  final.points[28].point.longitudinal_velocity_mps=0;
  EXPECT_THROW(lc::frenet_observation_hypothesis(final,f.native(),pose(25),10,3,.5,2,4),std::logic_error);
}
TEST(FrenetSafety, WrongDirectionOutOfCoverageAndNonmonotoneGeometryFailClosed){
  Fixture f(2);auto reverse=pose(25);reverse.orientation.w=0;reverse.orientation.z=1;
  EXPECT_THROW(lc::frenet_observation_hypothesis(path(),f.native(),reverse,10,3,.5,2,4),std::logic_error);
  auto native=f.native();native.front().pose.position.x=30;
  EXPECT_THROW(lc::frenet_observation_hypothesis(path(),native,pose(25),10,3,.5,2,4),std::logic_error);
  EXPECT_THROW(lc::frenet_observation_hypothesis(path(),f.native(),pose(-5),10,3,.5,2,4),std::logic_error);
}
// Regression: a cropped-away history is not a future geometry mismatch.
TEST(FrenetSafety, CroppedPastPrefixPreservesCurvedFuturePrediction){
  auto full=path();
  for(auto & point:full.points){
    auto & p=point.point.pose;const double x=p.position.x;
    p.position.y=.003*x*x;const double yaw=std::atan(.006*x);
    p.orientation.z=std::sin(yaw/2);p.orientation.w=std::cos(yaw/2);
  }
  lc::SafetyPrediction native;
  for(int i=0;i<=70;i+=5)native.emplace_back(i/10.0,full.points[i].point.pose,10);
  auto cropped=full;cropped.points.erase(cropped.points.begin(),cropped.points.begin()+15);
  const auto ego=full.points[25].point.pose;
  const auto expected=lc::frenet_observation_hypothesis(full,native,ego,10,3,.5,2,4);
  const auto actual=lc::frenet_observation_hypothesis(cropped,native,ego,10,3,.5,2,4);
  ASSERT_EQ(expected.size(),actual.size());
  for(size_t i=0;i<actual.size();++i){
    EXPECT_NEAR(expected[i].time,actual[i].time,1e-9);
    EXPECT_NEAR(expected[i].pose.position.x,actual[i].pose.position.x,1e-9);
    EXPECT_NEAR(expected[i].pose.position.y,actual[i].pose.position.y,1e-9);
    EXPECT_NEAR(expected[i].velocity,actual[i].velocity,1e-9);
  }
  cropped.points[25].point.pose.position.y+=2;
  EXPECT_THROW(lc::frenet_observation_hypothesis(cropped,native,ego,10,3,.5,2,4),std::logic_error);
}
TEST(FrenetSafety, NativeStopPlateauIsNotSkippedByTimeProjection){
  Fixture f(0);auto native=f.native();
  native.insert(native.begin()+5,{2.25,native[4].pose,0});
  native[4].velocity=0;
  EXPECT_THROW(lc::frenet_observation_hypothesis(path(),native,pose(15),10,3,.5,2,4),std::logic_error);
}
TEST(FrenetSafety, RemainingPrepareUsesEachHypothesisCurrentTime){
  const lc::SafetyPrediction fast{{0,pose(5),10},{1,pose(15),10},{3,pose(35),10}};
  const lc::SafetyPrediction slow{{0,pose(5),5},{1,pose(10),5},{3,pose(20),5}};
  EXPECT_DOUBLE_EQ(lc::remaining_prepare_time(path(),10,fast,false),.5);
  EXPECT_DOUBLE_EQ(lc::remaining_prepare_time(path(),10,slow,false),1);
  EXPECT_DOUBLE_EQ(lc::remaining_prepare_time(path(),5,fast,false),0);
  EXPECT_DOUBLE_EQ(lc::remaining_prepare_time(path(),3,fast,false),0);
  EXPECT_DOUBLE_EQ(lc::remaining_prepare_time(path(),50,fast,true),0);
}
TEST(FrenetSafety, StopBeforePrepareEndRemainsPrepareIncludingHorizonEndpoint){
  const lc::SafetyPrediction stopped{{0,pose(5),2},{1,pose(6),0},{3,pose(6),0}};
  EXPECT_GT(lc::remaining_prepare_time(path(),10,stopped,false),3);
}
TEST(FrenetSafety, RemainingPrepareRejectsReverseAndAmbiguousProjection){
  const lc::SafetyPrediction reverse{{0,pose(5),2},{1,pose(4),2},{3,pose(10),2}};
  EXPECT_THROW(lc::remaining_prepare_time(path(),10,reverse,false),std::logic_error);
  auto loop=path();auto extra=path();
  loop.points.insert(loop.points.end(),extra.points.begin(),extra.points.end());
  const lc::SafetyPrediction forward{{0,pose(5),2},{1,pose(7),2},{3,pose(11),2}};
  EXPECT_THROW(lc::remaining_prepare_time(loop,10,forward,false),std::logic_error);
}
