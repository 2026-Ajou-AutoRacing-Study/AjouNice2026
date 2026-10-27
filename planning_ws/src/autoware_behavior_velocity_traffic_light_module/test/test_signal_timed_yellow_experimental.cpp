// Copyright 2026 AjouNice2026 contributors
// Licensed under the Apache License, Version 2.0.
// Actual experimental scene boundaries with synthetic messages. This executable
// requires its own isolated ROS master; it never controls a simulator/vehicle.
#include "../src/experimental/scene.hpp"
#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
class TrafficTimedYellowExperimentalTest : public ::testing::Test
{
protected:
  using Element = autoware_perception_msgs::TrafficLightElement;
  using Signal = autoware_perception_msgs::TrafficLightGroup;
  ros::NodeHandle node_{"/traffic_timed_yellow_experimental_test"};
  std::shared_ptr<PlannerData> data_;
  std::shared_ptr<TrafficLightModule> module_;
  lanelet::TrafficLight::Ptr light_;
  std::shared_ptr<TrafficLightRos1Clock> clock_;
  std::shared_ptr<planning_factor_interface::PlanningFactorInterface> factors_;
  int64_t group_{201351};
  double now_{1000.};

  void SetUp() override
  {
    ros::Time::setNow(ros::Time(1000));
    for (const auto & value : std::vector<std::pair<std::string,double>>{
      {"wheel_radius",.3705},{"wheel_width",.2},{"wheel_base",3.},{"wheel_tread",1.6},
      {"front_overhang",.845},{"rear_overhang",1.},{"left_overhang",.2},{"right_overhang",.2},
      {"vehicle_height",1.6},{"max_steer_angle",.5},{"max_accel",-2.},
      {"max_jerk",-1.},{"system_delay",.2},{"delay_response_time",.2}}) node_.setParam(value.first,value.second);
    data_ = std::make_shared<PlannerData>(node_);
    data_->current_acceleration = std::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
    data_->is_simulation = false;
    data_->max_stop_acceleration_threshold=-6.; data_->max_stop_jerk_threshold=-10.;
    data_->delay_response_time=.5;
    clock_ = std::make_shared<TrafficLightRos1Clock>();
    factors_ = std::make_shared<planning_factor_interface::PlanningFactorInterface>(
      &node_,"traffic_timed_yellow_experimental_test");
  }

  void TearDown() override
  {
    module_.reset(); factors_.reset(); node_.deleteParam("/traffic_timed_yellow_experimental_test");
  }

  void configure(int64_t group, bool master=true, bool timed=true, int confirmations=3,
                 double min_yellow=2., double stop_margin=0., bool fail_open=true,
                 double judging_x=10., bool original_available=true, bool far_unseen=false,
                 bool initial_unseen=false, double unknown_grace_sec=0.)
  {
    module_.reset(); group_=group;
    lanelet::LineString3d line(lanelet::utils::getId(), {
      lanelet::Point3d(lanelet::utils::getId(),10.,-3.,0.),
      lanelet::Point3d(lanelet::utils::getId(),10.,3.,0.)});
    light_=lanelet::TrafficLight::make(group,{}, {line});
    lanelet::Lanelet lane(group+1000);
    lane.attributes()["turn_direction"]=group==201326 ? "left" : "straight";
    TrafficLightModule::PlannerParam params{};
    params.stop_margin=stop_margin;
    params.tl_state_timeout=.5; params.yellow_lamp_period=2.75;
    params.yellow_light_stop_velocity=1.; params.stop_time_hysteresis=.1;
    params.enable_pass_judge=true; params.yellow_light_is_go=true;
    params.competition_fail_open=fail_open; params.enable_signal_compliance_guard=true;
    params.defer_far_unseen_unknown=far_unseen;
    params.defer_unseen_until_observed=initial_unseen;
    params.enable_yellow_passthrough=master;
    params.signal_compliance_enable_timed_yellow=timed;
    params.signal_compliance_required_green_observations=confirmations;
    params.signal_compliance_unknown_grace_sec=unknown_grace_sec;
    params.signal_compliance_min_yellow_sec=min_yellow;
    params.signal_compliance_yellow_time_margin_sec=.2;
    params.signal_compliance_front_axle_offset_m=3.;
    params.use_official_judging_lines=true;
    if(original_available) params.signal_judging_lines={{group,{{judging_x,-3.},{judging_x,3.}},{1.,0.}}};
    module_=std::make_shared<TrafficLightModule>(group+1000,*light_,lane,line,
      group==201326,true,params,node_,"traffic_timed_yellow_experimental_test",clock_,nullptr,factors_);
    module_->setRTCEnabled(false);
    data_->traffic_light_id_map_raw_.clear();
  }

  void tick(double dt=.1)
  {
    now_+=dt; ros::Time t; t.fromSec(now_); ros::Time::setNow(t);
  }

  void pose(double rear_x=3., double speed=5.)
  {
    nav_msgs::Odometry m; m.header.stamp=ros::Time::now();m.header.frame_id="map";
    m.pose.pose.position.x=rear_x;m.pose.pose.orientation.w=1.;m.twist.twist.linear.x=speed;
    data_->updateOdometry(m);
  }

  Element lamp(uint8_t color,uint8_t shape=Element::CIRCLE)
  {
    Element e;e.color=color;e.shape=shape;e.status=Element::SOLID_ON;e.confidence=1.;return e;
  }

  Signal green()
  {
    Signal s;
    if (group_==201326) s.elements={lamp(Element::RED),lamp(Element::GREEN,Element::LEFT_ARROW)};
    else if (group_==201290) s.elements={lamp(Element::GREEN),lamp(Element::GREEN,Element::LEFT_ARROW)};
    else s.elements={lamp(Element::GREEN)};
    return s;
  }

  Signal yellow()
  {
    Signal s;s.elements={lamp(Element::AMBER)};
    if (group_==201326) s.elements.push_back(lamp(Element::RED));return s;
  }

  Signal red() {Signal s;s.elements={lamp(Element::RED)};return s;}

  void put(Signal s)
  {
    s.traffic_light_group_id=group_;
    TrafficSignalStamped m;m.stamp=ros::Time::now();m.signal=s;data_->traffic_light_id_map_raw_[group_]=m;
  }

  bool stops(double start=-30.)
  {
    std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> points;
    for(double x=start;x<=40.;x+=.25) {
      autoware_internal_planning_msgs::PathPointWithLaneId p;
      p.point.pose.position.x=x;p.point.pose.orientation.w=1.;
      p.point.longitudinal_velocity_mps=8.;p.lane_ids={group_+1000};points.push_back(p);
    }
    auto built=Trajectory::Builder{}.build(points);EXPECT_TRUE(built);if(!built)return true;
    auto trajectory=*built;geometry_msgs::Point a,b,c,d;
    a.x=c.x=start;b.x=d.x=40.;a.y=b.y=3.;c.y=d.y=-3.;
    EXPECT_TRUE(module_->modifyPathVelocity(trajectory,{a,b},{c,d},*data_));
    auto p=trajectory.restore();
    return std::any_of(p.begin(),p.end(),[](const auto & q){return q.point.longitudinal_velocity_mps<.001;});
  }

  void confirmedGreen(double rear_x=3.,double speed=5.,int count=3)
  {
    for(int n=0;n<count;++n) {
      tick();pose(rear_x,speed);put(green());EXPECT_EQ(stops(),n<count-1);
    }
  }
};

TEST_F(TrafficTimedYellowExperimentalTest, ABCDFarUnseenEmptyOpensPathButRedAndNearEmptyStop)
{
  for(const auto group:{201351,201365,201290,201326}) {
    SCOPED_TRACE(group);
    Signal empty;
    configure(group,false,false,3,2.,0.,true,10.,true,true);
    tick();pose(-65.,5.);put(empty);EXPECT_FALSE(stops(-100.));
    tick();put(red());EXPECT_TRUE(stops(-100.));
    tick();put(empty);EXPECT_TRUE(stops(-100.));

    configure(group,false,false,3,2.,0.,true,10.,true,true);
    tick();pose(-65.,5.);put(green());EXPECT_TRUE(stops(-100.));
    tick();put(green());EXPECT_TRUE(stops(-100.));
    tick();put(green());EXPECT_FALSE(stops(-100.));
    tick();put(empty);EXPECT_TRUE(stops(-100.));

    configure(group,false,false,3,2.,0.,true,10.,true,true);
    tick();pose(-20.,5.);put(empty);EXPECT_TRUE(stops(-100.));
  }
}

TEST_F(TrafficTimedYellowExperimentalTest, UnknownGraceABCDConfirmedOnlyAndOriginalLeaseExpiry)
{
  for (const auto group : {201351,201365,201290,201326}) {
    for (const int count : {1,2}) {
      configure(group,false,false,count,2.,0.,false,10.,true,false,false,.2);
      pose(0.,2.);
      for (int n=0; n<count; ++n) {
        tick(.01); put(green()); EXPECT_EQ(stops(),n<count-1);
      }
      Signal empty;
      tick(.05); put(empty); EXPECT_FALSE(stops());
      tick(.05); put(empty); EXPECT_FALSE(stops());
      tick(.101); put(empty); EXPECT_TRUE(stops());
    }
  }
}

TEST_F(TrafficTimedYellowExperimentalTest, UnknownGraceKnownColorMarkersAndNewRedYellowNeverPass)
{
  for (const auto color : {Element::RED,Element::AMBER,Element::GREEN}) {
    configure(201351,false,false,1,2.,0.,false,10.,true,false,false,.2);
    pose(0.,2.); tick(.01); put(green()); EXPECT_FALSE(stops());
    Signal empty;
    tick(.02); put(empty); EXPECT_FALSE(stops());
    Signal evidence; auto marker=lamp(color,Element::UNKNOWN); marker.status=Element::UNKNOWN;
    evidence.elements={marker};
    tick(.02); put(evidence); EXPECT_TRUE(stops());
    tick(.02); put(empty); EXPECT_TRUE(stops());
  }
  for (const bool is_yellow : {false,true}) {
    configure(201351,false,false,1,2.,0.,false,10.,true,false,false,.2);
    pose(0.,2.); tick(.01); put(green()); EXPECT_FALSE(stops());
    tick(.02); put(is_yellow ? yellow() : red()); EXPECT_TRUE(stops());
  }
}

TEST_F(TrafficTimedYellowExperimentalTest, UnknownGraceMissingAndUnconfirmedWrongDirectionDoNotRetainGo)
{
  Signal empty;
  configure(201351,false,false,2,2.,0.,false,10.,true,false,false,.2);
  pose(0.,2.); tick(.01); put(green()); EXPECT_TRUE(stops());
  tick(.02); put(empty); EXPECT_TRUE(stops());
  configure(201326,false,false,1,2.,0.,false,10.,true,false,false,.2);
  pose(0.,2.); Signal circle; circle.elements={lamp(Element::GREEN)};
  tick(.01); put(circle); EXPECT_TRUE(stops());
  tick(.02); put(empty); EXPECT_TRUE(stops());
  tick(.01); put(green()); EXPECT_FALSE(stops());
  data_->traffic_light_id_map_raw_.clear(); tick(.02); EXPECT_TRUE(stops());
  tick(.02); put(empty); EXPECT_TRUE(stops());
}

TEST_F(TrafficTimedYellowExperimentalTest, InitialUnseenStaysOpenNearLineUntilReadableLamp)
{
  for (const auto group : {201351,201365,201290,201326}) {
    SCOPED_TRACE(group);
    Signal empty;
    configure(group,false,false,3,2.,0.,true,10.,true,false,true);
    tick();pose(-20.,5.);put(empty);EXPECT_FALSE(stops(-100.));
    tick();put(red());EXPECT_TRUE(stops(-100.));
    tick();put(empty);EXPECT_TRUE(stops(-100.));
  }
}

TEST_F(TrafficTimedYellowExperimentalTest, YellowUsesCameraTargetNotFartherOriginalCrossingLine)
{
  for(const auto group:{201351,201365,201290,201326}) {
    configure(group,true,true,3,2.,0.,true,20.);
    confirmedGreen(0.,5.); tick(); pose(0.,5.); put(yellow());
    // Bumper target6.155m is beyond the simple stopping distance4.0625m: STOP.
    EXPECT_TRUE(stops());
    configure(group,true,true,3,2.,0.,true,20.);
    confirmedGreen(10.,5.); tick(); pose(10.,5.); put(yellow());
    // Camera stop target is behind ego, original line not crossed: retain GO.
    EXPECT_FALSE(stops());
  }
}

TEST_F(TrafficTimedYellowExperimentalTest, MissingOriginalLineStillUsesLaneletYellowDecision)
{
  configure(201351,true,true,3,2.,0.,true,20.,false);
  confirmedGreen(); tick(); pose(); put(yellow());
  EXPECT_FALSE(stops());
}

TEST_F(TrafficTimedYellowExperimentalTest, ABCDObsoleteTimedSwitchCannotChangeNewDecision)
{
  // Rear3 -> Lanelet bumper target3.155m. Simple stop4.0625m: GO,
  // independently of the obsolete timing switch and its former ETA policy.
  for(const auto group:{201351,201365,201290,201326}) {
    SCOPED_TRACE(group);
    for(const bool timed:{false,true}) {
      configure(group,true,timed);confirmedGreen();tick();pose();put(yellow());
      EXPECT_FALSE(stops());
    }
  }
}

TEST_F(TrafficTimedYellowExperimentalTest, MasterOffOverridesTimeCandidateForAllMissions)
{
  for(const auto group:{201351,201365,201290,201326}) {
    configure(group,false,true);confirmedGreen();tick();pose();put(yellow());EXPECT_TRUE(stops());
  }
}

TEST_F(TrafficTimedYellowExperimentalTest, ABCDFirstYellowAndStoppedYellowNeverStart)
{
  for(const auto group:{201351,201365,201290,201326}) {
    configure(group);tick();pose(6.,5.);put(yellow());EXPECT_TRUE(stops());
    configure(group);confirmedGreen(6.,0.);tick();pose(6.,0.);put(yellow());EXPECT_TRUE(stops());
  }
}

TEST_F(TrafficTimedYellowExperimentalTest, ABCDPriorStopCannotBecomeYellowDepartureCloserToTarget)
{
  for (const auto group : {201351,201365,201290,201326}) {
    configure(group); confirmedGreen(0.,5.);
    tick(); pose(0.,5.); put(yellow()); EXPECT_TRUE(stops());
    tick(); pose(6.,5.); put(yellow()); EXPECT_TRUE(stops());
  }
}

TEST_F(TrafficTimedYellowExperimentalTest, ABCDUnknownGracePreservesOnlyPriorGrantedTransition)
{
  for (const auto group : {201351,201365,201290,201326}) {
    for (const bool expired : {false,true}) {
      configure(group,true,true,1,2.,0.,false,20.,true,false,false,.3);
      tick(); pose(3.,5.); put(green()); EXPECT_FALSE(stops());
      Signal empty; tick(expired ? .31 : .05); put(empty); EXPECT_EQ(stops(),expired);
      tick(.05); put(yellow()); EXPECT_EQ(stops(),expired);
    }
  }
}

TEST_F(TrafficTimedYellowExperimentalTest, ABCDRedPastCameraTargetStillStopsBeforeOfficialCrossing)
{
  for (const auto group : {201351,201365,201290,201326}) {
    configure(group,true,true,1,2.,0.,false,20.);
    tick(); pose(6.5,2.); put(green()); EXPECT_FALSE(stops());
    tick(); put(yellow()); EXPECT_FALSE(stops());
    tick(); put(red()); EXPECT_TRUE(stops());
    EXPECT_EQ(module_->getTrafficLightModuleState(),TrafficLightModule::State::APPROACH);
  }
}

TEST_F(TrafficTimedYellowExperimentalTest, ObsoleteYellowDurationsNoLongerAffectDecision)
{
  for(const double minimum:{1.,2.,3.}) {
    configure(201351,true,true,3,minimum);confirmedGreen();tick();pose();put(yellow());
    EXPECT_FALSE(stops());
  }
}

TEST_F(TrafficTimedYellowExperimentalTest, TwoDistinctGreenFramesReleaseAndRepeatedSourceDoesNot)
{
  configure(201351,true,true,2);tick();pose();put(green());EXPECT_TRUE(stops());
  for(int repeat=0;repeat<10;++repeat) {tick(.01);pose();EXPECT_TRUE(stops());}
  tick();pose();put(green());EXPECT_FALSE(stops());
  tick();pose();put(yellow());EXPECT_FALSE(stops());
}

TEST_F(TrafficTimedYellowExperimentalTest, DStraightGreenIsNotLeftAuthorization)
{
  configure(201326);
  Signal straight;straight.elements={lamp(Element::GREEN)};
  for(int n=0;n<3;++n){tick();pose(6.,5.);put(straight);EXPECT_TRUE(stops());}
  tick();pose(6.,5.);put(yellow());EXPECT_TRUE(stops());
}

TEST_F(TrafficTimedYellowExperimentalTest, RedMissingAndStaleInvalidateAnEarlierTimedPass)
{
  for(int fault=0;fault<3;++fault) {
    configure(201351,true,true,3,2.,0.,false);
    confirmedGreen();tick();pose();put(yellow());ASSERT_FALSE(stops());
    if(fault==0){tick();pose();put(red());}
    if(fault==1){data_->traffic_light_id_map_raw_.clear();tick();pose();}
    if(fault==2){tick(.6);pose();}
    EXPECT_TRUE(stops());
  }
}

TEST_F(TrafficTimedYellowExperimentalTest, FreshYellowDoesNotUseAnAssumedElapsedTimeDeadline)
{
  configure(201351);confirmedGreen();tick();pose();put(yellow());ASSERT_FALSE(stops());
  // Process every fresh frame: skipping the callbacks would intentionally
  // exceed the separate .5s observation timeout, not a yellow phase deadline.
  for(int n=0;n<10;++n){tick();pose();put(yellow());EXPECT_FALSE(stops());}
  EXPECT_FALSE(stops()); // Artificial fixed pose/speed: same braking geometry, no duration model.
}

TEST_F(TrafficTimedYellowExperimentalTest, CurrentAccelerationDoesNotChangeSimpleBrakingApproximation)
{
  configure(201351);confirmedGreen();
  auto braking=std::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
  braking->accel.accel.linear.x=-3.;data_->current_acceleration=braking;
  tick();pose();put(yellow());EXPECT_FALSE(stops());
}

TEST_F(TrafficTimedYellowExperimentalTest, AlreadyCrossedClearsRedEvenWithMasterOff)
{
  for(const auto group:{201351,201365,201290,201326}) {
    configure(group,false,true);tick();pose(7.1,0.);put(red());EXPECT_FALSE(stops());
    EXPECT_EQ(module_->getTrafficLightModuleState(),TrafficLightModule::State::GO_OUT);
  }
}
TEST_F(TrafficTimedYellowExperimentalTest, StandardMarginAppliesToAllMissionsWithoutMovingCrossingLine)
{
  for (const auto group : {201351, 201365, 201290, 201326}) {
    SCOPED_TRACE(group);
    configure(group,false,false,3,2.,.75);
    tick();pose(0.,0.);put(red());
    ASSERT_TRUE(stops());
    ASSERT_TRUE(module_->getFirstStopPointArcLength());
    EXPECT_NEAR(*module_->getFirstStopPointArcLength(),30.+10.-3.845-.75,1e-5);
    tick();pose(6.99,0.);put(red());
    EXPECT_TRUE(stops());
    EXPECT_EQ(module_->getTrafficLightModuleState(),TrafficLightModule::State::APPROACH);
    tick();pose(7.01,0.);put(red());
    EXPECT_FALSE(stops());
    EXPECT_EQ(module_->getTrafficLightModuleState(),TrafficLightModule::State::GO_OUT);
  }
}
}  // namespace autoware::behavior_velocity_planner::experimental

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"traffic_timed_yellow_experimental_gtest");
  testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
