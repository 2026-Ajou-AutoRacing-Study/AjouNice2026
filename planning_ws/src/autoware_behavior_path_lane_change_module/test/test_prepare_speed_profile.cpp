#include <autoware/behavior_path_lane_change_module/utils/prepare_speed_profile.hpp>
#include <autoware/behavior_path_lane_change_module/utils/lateral_speed_cap.hpp>
#include <autoware/behavior_path_lane_change_module/structs/data.hpp>
#include <autoware/behavior_path_lane_change_module/utils/utils.hpp>
#include <gtest/gtest.h>
using namespace autoware::behavior_path_planner::utils::lane_change;

TEST(PrepareProfile, GenerationAndRefreshShareObservedLateralCap) {
  autoware_internal_planning_msgs::PathWithLaneId path;
  constexpr double curvature=.017551297484834125;
  const double radius=1/curvature;
  for(double angle:{-.05,0.,.05}) {
    autoware_internal_planning_msgs::PathPointWithLaneId p;
    p.point.pose.position.x=radius*std::sin(angle);
    p.point.pose.position.y=radius*(1-std::cos(angle));
    p.point.longitudinal_velocity_mps=90.60551834106445/3.6;
    path.points.push_back(p);
  }
  const double cap=lateral_speed_cap(path.points,1,4.5);
  EXPECT_NEAR(cap*3.6,57.64400458,1e-5);
  EXPECT_NEAR(cap*cap*curvature,4.5,1e-10);
  EXPECT_LT(cap,path.points[1].point.longitudinal_velocity_mps);
  EXPECT_THROW(lateral_speed_cap(path.points,1,0),std::logic_error);
}

TEST(PrepareProfile, LateralCapPropagatesThroughPreparationWithoutClampingInitialSpeed) {
  const std::vector<double> s{0,10,20,30,40,50,60};
  const double cap=std::sqrt(4.5/.017551297484834125);
  const std::vector<double> limits{30,30,30,cap,30,30,30};
  const auto v=make_maneuver_speed_profile(s,limits,18,20,3.25,3.25,4);
  EXPECT_DOUBLE_EQ(v.front(),18);
  EXPECT_LE(v[3],cap);
  for(size_t i=1;i<v.size();++i) {
    const double a=(v[i]*v[i]-v[i-1]*v[i-1])/(2*(s[i]-s[i-1]));
    EXPECT_LE(a,3.25+1e-8);EXPECT_GE(a,-4-1e-8);
  }
  EXPECT_THROW(make_maneuver_speed_profile({0,1,2},{30,cap,30},25,1,3.25,3.25,4),std::logic_error);
}
TEST(PrepareProfile, RoadCapIsNotInstant150Kph) {
  std::vector<double>s{0,2,5,10,15,20,30,40,60};
  auto v=make_maneuver_speed_profile(s,std::vector<double>(s.size(),41.6667),16,20,-2,3.25,6);
  EXPECT_DOUBLE_EQ(v.front(),16);
  for(size_t i=1;i<v.size();++i) {
    const double a=(v[i]*v[i]-v[i-1]*v[i-1])/(2*(s[i]-s[i-1]));
    EXPECT_LE(a,3.25+1e-8);EXPECT_GE(a,-6-1e-8);
    if(s[i]<=20) EXPECT_LE(v[i],16);
  }
  EXPECT_NEAR(v[5],std::sqrt(176),1e-8);
  EXPECT_LT(sample_speed_profile(s,v,.5).second,16);
}
TEST(PrepareProfile, CalibratedAccelerationAndRoadLimit) {
  autoware::motion_utils::AccelerationLimits map({{0,1},{40,.5}});
  std::vector<double>s{0,10,20,30,40};
  auto v=make_maneuver_speed_profile(s,{30,30,30,12,12},10,10,3,3,6,map);
  for(size_t i=1;i<v.size();++i) {
    EXPECT_LE(v[i],12);
    EXPECT_LE((v[i]*v[i]-v[i-1]*v[i-1])/(2*(s[i]-s[i-1])),map.at(v[i])+1e-8);
  }
}
TEST(PrepareProfile, StopPreservedAndImpossibleDecelerationRejected) {
  auto v=make_maneuver_speed_profile({0,20,40},{20,20,0},10,0,0,3,6);
  EXPECT_EQ(v.back(),0);
  EXPECT_THROW(make_maneuver_speed_profile({0,1,2},{20,20,0},15,0,0,3,6),std::logic_error);
}
TEST(PrepareProfile, AnchorLeavesPrefixAndShiftedIndexDomainUnchanged) {
  autoware_internal_planning_msgs::PathWithLaneId path;
  for(int i=0;i<5;++i) {
    autoware_internal_planning_msgs::PathPointWithLaneId p;
    p.point.pose.position.x=i*10;p.point.pose.orientation.w=1;
    p.point.longitudinal_velocity_mps=i==4?0:41.6667;p.lane_ids={101};path.points.push_back(p);
  }
  auto shifted=path;auto before=path;geometry_msgs::Pose anchor;anchor.position.x=15;anchor.orientation.w=1;
  const auto idx=insert_maneuver_anchor(path,1,anchor);
  ASSERT_EQ(idx,2);EXPECT_EQ(path.points.size(),6);EXPECT_EQ(shifted,before);
  EXPECT_EQ(path.points[0],before.points[0]);EXPECT_EQ(path.points[1],before.points[1]);
  EXPECT_EQ(path.points.back(),before.points.back());EXPECT_EQ(path.points[idx].lane_ids,before.points[1].lane_ids);
  EXPECT_EQ(insert_maneuver_anchor(path,1,anchor),2);EXPECT_EQ(path.points.size(),6);
}
TEST(PrepareProfile, ExactAnchorTimeZeroDoesNotUseBackwardRoadCap) {
  autoware_internal_planning_msgs::PathWithLaneId path;
  for(int i=0;i<6;++i) {
    autoware_internal_planning_msgs::PathPointWithLaneId p;
    p.point.pose.position.x=i*10;p.point.pose.orientation.w=1;
    p.point.longitudinal_velocity_mps=41.6667;p.lane_ids={101};path.points.push_back(p);
  }
  geometry_msgs::Pose pose;pose.position.x=15;pose.orientation.w=1;
  const auto anchor=insert_maneuver_anchor(path,1,pose);
  auto v=make_maneuver_speed_profile({0,5,15,25},{41.6667,41.6667,41.6667,41.6667},14,15,-1,3,6);
  for(size_t i=0;i<v.size();++i)path.points[anchor+i].point.longitudinal_velocity_mps=v[i];
  EXPECT_FLOAT_EQ(path.points[anchor-1].point.longitudinal_velocity_mps,41.6667);
  EXPECT_FLOAT_EQ(path.points[anchor].point.longitudinal_velocity_mps,14);
  EXPECT_DOUBLE_EQ(sample_speed_profile({0,5,15,25},v,0).second,14);
  EXPECT_LT(sample_speed_profile({0,5,15,25},v,.5).second,14);
  EXPECT_FLOAT_EQ(path.points.back().point.longitudinal_velocity_mps,41.6667);
}
TEST(PrepareProfile, RealCollisionPredictorConsumesAnchoredPublishedProfile) {
  using namespace autoware::behavior_path_planner;
  auto data=std::make_shared<lane_change::CommonData>();
  data->lc_param_ptr=std::make_shared<lane_change::Parameters>();
  data->bpp_param_ptr=std::make_shared<BehaviorPathPlannerParameters>();
  data->lc_param_ptr->safety.collision_check.prediction_time_resolution=.5;
  data->bpp_param_ptr->ego_nearest_dist_threshold=3;
  data->bpp_param_ptr->ego_nearest_yaw_threshold=1;
  auto odom=boost::make_shared<nav_msgs::Odometry>();
  odom->pose.pose.position.x=15;odom->pose.pose.orientation.w=1;
  odom->twist.twist.linear.x=14;data->self_odometry_ptr=odom;
  LaneChangePath candidate;candidate.type=lane_change::PathType::SpatialSpeedProfile;
  for(int i=0;i<=10;++i) {
    PathPointWithLaneId p;p.point.pose.position.x=i*10;p.point.pose.orientation.w=1;
    p.point.longitudinal_velocity_mps=41.6667;p.lane_ids={10};candidate.path.points.push_back(p);
  }
  const auto anchor=insert_maneuver_anchor(candidate.path,1,odom->pose.pose);
  std::vector<double>s;
  for(size_t i=anchor;i<candidate.path.points.size();++i)s.push_back(candidate.path.points[i].point.pose.position.x-15);
  auto v=make_maneuver_speed_profile(s,std::vector<double>(s.size(),41.6667),14,15,-1,3,6);
  for(size_t i=0;i<v.size();++i)candidate.path.points[anchor+i].point.longitudinal_velocity_mps=v[i];
  candidate.info.lane_changing_end=candidate.path.points.back().point.pose;
  auto predictions=convert_to_predicted_paths(data,candidate,1,false);
  ASSERT_EQ(predictions.size(),1);ASSERT_FALSE(predictions.front().empty());
  EXPECT_NEAR(predictions[0][0].pose.position.x,15,1e-6);
  EXPECT_NEAR(predictions[0][0].velocity,14,1e-6);
  for(const auto & p:predictions[0]) {
    auto expected=sample_speed_profile(s,v,p.time);
    EXPECT_NEAR(p.pose.position.x,15+expected.first,1e-4);
    EXPECT_NEAR(p.velocity,expected.second,1e-4);
  }
  EXPECT_NEAR(predictions[0].back().time,speed_profile_duration(s,v),1e-4);
  EXPECT_NEAR(predictions[0].back().pose.position.x,100,1e-5);
}
