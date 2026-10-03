#include <autoware/behavior_path_lane_change_module/utils/rss_phase.hpp>
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/safety_core.hpp>
#include <gtest/gtest.h>
#include <limits>
#include <array>

namespace checker = autoware::behavior_path_planner::utils::path_safety_checker;
using autoware::behavior_path_planner::utils::lane_change::check_rss_phases;
using checker::ExtendedPredictedObject;
using checker::PoseWithVelocityStamped;
using checker::PredictedPathWithPolygon;
#include "rss_phase_r14_fixture.hpp"
namespace
{
ExtendedPredictedObject object_at_times(const std::vector<double> & times)
{
  ExtendedPredictedObject object;
  object.uuid.uuid[0] = 1;
  PredictedPathWithPolygon prediction;
  for (const auto time : times) {
    checker::PoseWithVelocityAndPolygonStamped point;
    point.time = time;
    point.pose.orientation.w = 1;
    prediction.path.push_back(point);
  }
  object.predicted_paths.push_back(prediction);
  return object;
}
std::vector<PoseWithVelocityStamped> ego_times()
{
  return {{0,geometry_msgs::Pose{},0},{1,geometry_msgs::Pose{},0},{2,geometry_msgs::Pose{},0}};
}
}

TEST(RssPhase, ExactBoundaryAndEverySampleOnce)
{
  const auto object = object_at_times({0,.5,1,1.5,2});
  std::vector<std::pair<double,bool>> visits;
  std::string reason;
  ASSERT_TRUE(check_rss_phases(object,ego_times(),1.0,
    [&](const auto & phase,bool prepare,const auto & original) {
      EXPECT_EQ(original.path.size(),5U);
      for(const auto & p:phase.predicted_paths.front().path) visits.emplace_back(p.time,prepare);
      return true;
    },reason));
  EXPECT_EQ(visits,(std::vector<std::pair<double,bool>>{{0,true},{.5,true},{1,false},{1.5,false},{2,false}}));
}
TEST(RssPhase, ZeroPreparationAndEntirelyLatePrediction)
{
  for(const double duration:{0.0,.5}) {
    int samples=0;std::string reason;
    EXPECT_TRUE(check_rss_phases(object_at_times({1,2}),ego_times(),duration,
      [&](const auto & phase,bool prepare,const auto &) {
        EXPECT_FALSE(prepare);samples+=phase.predicted_paths.front().path.size();return true;
      },reason));
    EXPECT_EQ(samples,2);
  }
}
TEST(RssPhase, MultipleHypothesesAndFirstUnsafePreserved)
{
  auto object=object_at_times({0,1,2});
  object.predicted_paths.push_back(object.predicted_paths.front());
  std::vector<std::string> calls;std::string reason;int hypothesis=0;
  EXPECT_FALSE(check_rss_phases(object,ego_times(),1,
    [&](const auto &,bool prepare,const auto &) {
      if(prepare)++hypothesis;
      calls.push_back(std::to_string(hypothesis)+(prepare?"prepare":"changing"));
      return !(hypothesis==2 && prepare);
    },reason));
  EXPECT_EQ(calls,(std::vector<std::string>{"1prepare","1changing","2prepare"}));
  EXPECT_EQ(reason,"rss_prepare_unsafe");
}
TEST(RssPhase, InvalidTimesFailBeforeAnyCoreCall)
{
  const auto nan=std::numeric_limits<double>::quiet_NaN();
  int calls=0;std::string reason;
  const auto check=[&](const auto &,bool,const auto &){++calls;return true;};
  for(double duration:{-1.0,nan})
    EXPECT_FALSE(check_rss_phases(object_at_times({0,1}),ego_times(),duration,check,reason));
  for(const auto & times:std::vector<std::vector<double>>{{-1,0},{0,nan},{0,0},{1,0},{}})
    EXPECT_FALSE(check_rss_phases(object_at_times(times),ego_times(),1,check,reason));
  auto object=object_at_times({0,1});object.predicted_paths.push_back(object_at_times({0,nan}).predicted_paths[0]);
  EXPECT_FALSE(check_rss_phases(object,ego_times(),1,check,reason));
  auto ego=ego_times();ego[1].time=nan;
  EXPECT_FALSE(check_rss_phases(object_at_times({0,1}),ego,1,check,reason));
  EXPECT_EQ(calls,0);
}

TEST(RssPhase, RealCorePreparationDoesNotApplyCancelEarly)
{
  // R14/001 object 0x1f at predicted t=1.5 < prepare 1.505897.
  // Frozen relative kinematics: ego 25.992m/s, front 13.895m/s, ~127m ahead.
  // Use aligned straight-lane boxes to isolate the selector from map geometry.
  autoware_internal_planning_msgs::PathWithLaneId path;
  for(double x:{0.0,200.0}) {
    autoware_internal_planning_msgs::PathPointWithLaneId p;
    p.point.pose.position.x=x;p.point.pose.orientation.w=1;path.points.push_back(p);
  }
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.max_longitudinal_offset_m=3.8;vehicle.min_longitudinal_offset_m=-1;
  vehicle.max_lateral_offset_m=1;vehicle.min_lateral_offset_m=-1;
  vehicle.vehicle_width_m=2;vehicle.rear_overhang_m=1;
  vehicle.wheel_base_m=3;vehicle.front_overhang_m=.8;
  vehicle.wheel_tread_m=1.8;vehicle.left_overhang_m=.1;vehicle.right_overhang_m=.1;
  geometry_msgs::Pose ego_pose;ego_pose.orientation.w=1;
  std::vector<PoseWithVelocityStamped> ego{{0,ego_pose,25.992240289953475},{2,ego_pose,25.992240289953475}};
  auto object=object_at_times({1.5});object.initial_pose.position.x=127;
  object.initial_pose.orientation.w=1;object.initial_twist.linear.x=13.895430591532827;
  auto & p=object.predicted_paths[0].path[0];
  p.pose=object.initial_pose;p.velocity=object.initial_twist.linear.x;
  for(const auto & xy:std::vector<std::pair<double,double>>{{124.6,-1},{124.6,1},{129.4,1},{129.4,-1},{124.6,-1}})
    p.polygon.outer().emplace_back(xy.first,xy.second);
  object.initial_polygon=p.polygon;
  checker::RSSparams prepare,cancel;
  prepare.extended_polygon_policy="rectangle";prepare.front_vehicle_deceleration=-6;
  prepare.rear_vehicle_deceleration=-6;prepare.rear_vehicle_reaction_time=.5;
  prepare.lateral_distance_max_threshold=.5;prepare.longitudinal_distance_min_threshold=1;
  cancel=prepare;cancel.front_vehicle_deceleration=-1;cancel.rear_vehicle_deceleration=-2;
  cancel.rear_vehicle_reaction_time=1.5;cancel.rear_vehicle_safety_time_margin=.8;
  cancel.lateral_distance_max_threshold=1;cancel.longitudinal_distance_min_threshold=2.5;
  checker::CollisionCheckDebugMap debug;
  const auto check=[&](const auto & candidate,bool is_prepare,const auto &) {
    return checker::check_safety_with_rss(path,ego,vehicle,{candidate},debug,
      is_prepare?prepare:cancel,true,1,3.14);
  };
  EXPECT_FALSE(check(object,false,object.predicted_paths[0]));
  EXPECT_FALSE(debug.at(object.uuid.uuid).is_safe);
  std::string reason;
  EXPECT_TRUE(check_rss_phases(object,ego,1.5058970517087764,check,reason));
  EXPECT_TRUE(check_rss_phases(object,ego,1.5000001,check,reason));
  EXPECT_FALSE(check_rss_phases(object,ego,1.5,check,reason));
  EXPECT_EQ(reason,"rss_lane_changing_unsafe");
  EXPECT_FALSE(debug.at(object.uuid.uuid).is_safe);
  EXPECT_EQ(debug.at(object.uuid.uuid).unsafe_reason,"overlap_extended_polygon");
}

TEST(RssPhase, FrozenR14CoreRetainsLaterUnsafeRatherThanClaimingAllClear)
{
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.max_longitudinal_offset_m=3.845;vehicle.min_longitudinal_offset_m=-.79;
  vehicle.vehicle_width_m=1.89;vehicle.rear_overhang_m=.79;
  vehicle.wheel_base_m=3;vehicle.front_overhang_m=.845;
  vehicle.wheel_tread_m=1.651;vehicle.left_overhang_m=.1195;vehicle.right_overhang_m=.1195;
  checker::RSSparams prepare,cancel;
  prepare.extended_polygon_policy="along_path";prepare.front_vehicle_deceleration=-6;
  prepare.rear_vehicle_deceleration=-6;prepare.rear_vehicle_reaction_time=.5;
  prepare.lateral_distance_max_threshold=.5;prepare.longitudinal_distance_min_threshold=1;
  cancel=prepare;cancel.extended_polygon_policy="rectangle";
  cancel.front_vehicle_deceleration=-1;cancel.rear_vehicle_deceleration=-2;
  cancel.rear_vehicle_reaction_time=1.5;cancel.rear_vehicle_safety_time_margin=.8;
  cancel.lateral_distance_max_threshold=1;cancel.longitudinal_distance_min_threshold=2.5;
  for(size_t i=0;i<3;++i) {
    SCOPED_TRACE(i);
    const auto fixture=frozen_r14(i);checker::CollisionCheckDebugMap debug;
    const auto check=[&](const auto & object,bool is_prepare,const auto & original) {
      const bool safe=checker::check_safety_with_rss(fixture.path,fixture.ego,vehicle,
        {object},debug,is_prepare?prepare:cancel,true,1,3.14);
      if(!safe)debug[object.uuid.uuid].object_predicted_path=original.path;
      return safe;
    };
    ASSERT_TRUE(check(fixture.object,true,fixture.object.predicted_paths.front()));
    ASSERT_FALSE(check(fixture.object,false,fixture.object.predicted_paths.front()));
    const auto unsafe_time=[&] {
      const auto & expected=debug.at(fixture.object.uuid.uuid).expected_ego_pose.position;
      auto closest=fixture.ego.begin();double distance=std::numeric_limits<double>::max();
      for(auto it=fixture.ego.begin();it!=fixture.ego.end();++it) {
        const double d=std::hypot(it->pose.position.x-expected.x,it->pose.position.y-expected.y);
        if(d<distance){distance=d;closest=it;}
      }
      return closest->time;
    };
    EXPECT_DOUBLE_EQ(unsafe_time(),i==0?2.0:1.5);
    std::string reason;
    const bool safe=check_rss_phases(fixture.object,fixture.ego,1.5058970517087764,check,reason);
    // Objects 0x1f/0x25 only collided under prematurely applied cancel RSS.
    // 0x1a remains unsafe at t=2s, so the whole candidate is NOT made safe.
    EXPECT_EQ(safe,i!=0);
    if(i==0) {
      EXPECT_EQ(reason,"rss_lane_changing_unsafe");
      EXPECT_DOUBLE_EQ(unsafe_time(),2.0);
      EXPECT_FALSE(debug.at(fixture.object.uuid.uuid).is_safe);
      EXPECT_EQ(debug.at(fixture.object.uuid.uuid).object_predicted_path.size(),
        fixture.object.predicted_paths.front().path.size());
    } else {
      EXPECT_EQ(reason,"safe");
      EXPECT_TRUE(debug.at(fixture.object.uuid.uuid).is_safe);
    }
  }
}
