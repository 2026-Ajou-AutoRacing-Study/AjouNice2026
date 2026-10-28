// Copyright 2026 AjouNice2026 contributors
// Licensed under the Apache License, Version 2.0.
// Deterministic actual-module tests: no simulator, native API, /clock publication
// or vehicle-command publisher. ros::Time::setNow affects this test process only.
#include "../src/scene.hpp"
#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace autoware::behavior_velocity_planner
{
class TrafficLightComplianceModuleTest : public ::testing::Test
{
protected:
  using Element = autoware_perception_msgs::TrafficLightElement;
  using Signal = autoware_perception_msgs::TrafficLightGroup;
  using Path = autoware_internal_planning_msgs::PathWithLaneId;
  ros::NodeHandle node_{"/traffic_light_compliance_test"};
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
      {"max_jerk",-1.},{"system_delay",.2},{"delay_response_time",.2}}) {
      node_.setParam(value.first,value.second);
    }
    data_ = std::make_shared<PlannerData>(node_);
    data_->current_acceleration = std::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
    data_->is_simulation = false;
    clock_ = std::make_shared<TrafficLightRos1Clock>();
    factors_ = std::make_shared<planning_factor_interface::PlanningFactorInterface>(
      &node_,"traffic_light_compliance_test");
    configure(201351);
  }

  void TearDown() override
  {
    module_.reset();
    factors_.reset();
    node_.deleteParam("/traffic_light_compliance_test");
  }

  void configure(int64_t group, bool guarded=true, bool yellow_passthrough=true,
                 double judging_x=10., bool original_available=true,
                 double unknown_grace_sec=0., int confirmations=3)
  {
    group_=group;
    lanelet::Points3d points{
      lanelet::Point3d(lanelet::utils::getId(),10.,-3.,0.),
      lanelet::Point3d(lanelet::utils::getId(),10.,3.,0.)};
    lanelet::LineString3d line(lanelet::utils::getId(),points);
    light_ = lanelet::TrafficLight::make(group,{}, {line});
    lanelet::Lanelet lane(group+1000);
    lane.attributes()["turn_direction"] = group==201326 ? "left" : "straight";
    TrafficLightModule::PlannerParam params{};
    // Pin the original numerical boundary fixtures; production defaults are
    // tested independently below and may be recalibrated without moving these scenes.
    params.signal_compliance_yellow_deceleration_mps2=8.;
    params.signal_compliance_yellow_jerk_mps3=16.;
    params.tl_state_timeout=1.; params.yellow_lamp_period=2.75;
    params.yellow_light_stop_velocity=1.; params.stop_time_hysteresis=.1;
    params.enable_pass_judge=true; params.yellow_light_is_go=true;
    // Existing dilemma/legacy tests are explicit ON comparisons, not defaults.
    params.enable_yellow_passthrough=yellow_passthrough;
    params.competition_fail_open=true; params.enable_signal_compliance_guard=guarded;
    params.signal_compliance_unknown_grace_sec=unknown_grace_sec;
    params.signal_compliance_required_green_observations=confirmations;
    params.signal_compliance_front_axle_offset_m=3.;
    params.use_official_judging_lines=true;
    if(original_available) params.signal_judging_lines={{group,{{judging_x,-3.},{judging_x,3.}},{1.,0.}}};
    module_ = std::make_shared<TrafficLightModule>(group+1000,*light_,lane,line,
      group==201326,true,params,node_,"traffic_light_compliance_test",clock_,nullptr,factors_);
    module_->setPlannerData(data_);
    module_->setRTCEnabled(false);
    data_->traffic_light_id_map_raw_.clear();
    pose(0.,0.);
  }

  void tick(double seconds=.1)
  {
    now_+=seconds;
    ros::Time value; value.fromSec(now_); ros::Time::setNow(value);
  }
  void pose(double x, double speed)
  {
    nav_msgs::Odometry message;
    message.header.stamp=ros::Time::now(); message.header.frame_id="map";
    message.pose.pose.position.x=x; message.pose.pose.orientation.w=1.;
    message.twist.twist.linear.x=speed;
    data_->updateOdometry(message);
  }
  Element lamp(uint8_t color, uint8_t shape=Element::CIRCLE)
  {
    Element result;
    result.color=color; result.shape=shape; result.status=Element::SOLID_ON; result.confidence=1.;
    return result;
  }
  Signal green()
  {
    Signal result;
    if(group_==201326) result.elements={lamp(Element::RED),lamp(Element::GREEN,Element::LEFT_ARROW)};
    else if(group_==201290) result.elements={lamp(Element::GREEN),lamp(Element::GREEN,Element::LEFT_ARROW)};
    else result.elements={lamp(Element::GREEN)};
    return result;
  }
  Signal yellow()
  {
    Signal result;
    result.elements={lamp(Element::AMBER)};
    if(group_==201326) result.elements.push_back(lamp(Element::RED));
    return result;
  }
  Signal red() { Signal result; result.elements={lamp(Element::RED)}; return result; }
  void put(Signal state, double age=0.)
  {
    state.traffic_light_group_id=group_;
    TrafficSignalStamped stamped;
    stamped.stamp=ros::Time::now()-ros::Duration(age); stamped.signal=state;
    data_->traffic_light_id_map_raw_[group_]=stamped;
  }
  Path path(double start=-30.)
  {
    Path result;
    result.header.frame_id="map"; result.header.stamp=ros::Time::now();
    for(double x=start;x<=40.;x+=.25) {
      autoware_internal_planning_msgs::PathPointWithLaneId point;
      point.point.pose.position.x=x; point.point.pose.orientation.w=1.;
      point.point.longitudinal_velocity_mps=8.; point.lane_ids={group_+1000};
      result.points.push_back(point);
    }
    geometry_msgs::Point a,b,c,d;
    a.x=c.x=start; b.x=d.x=40.; a.y=b.y=3.; c.y=d.y=-3.;
    result.left_bound={a,b}; result.right_bound={c,d};
    return result;
  }
  bool modifiesToStop(double path_start=-30.)
  {
    auto result=path(path_start);
    EXPECT_TRUE(module_->modifyPathVelocity(&result));
    return std::any_of(result.points.begin(),result.points.end(),[](const auto & point) {
      return point.point.longitudinal_velocity_mps<.001;
    });
  }
  void confirmGreenOnRealPath(double x=0., double speed=5.)
  {
    for(int index=0;index<3;++index) {
      tick(); pose(x,speed); put(green());
      EXPECT_EQ(modifiesToStop(),index<2);
    }
  }
  bool signalStop() { return module_->isStopSignal(
    10.-3.845-data_->current_odometry->pose.position.x); }
  bool pass(double bumper_target_distance) { return module_->isPassthrough(bumper_target_distance); }
  auto state() { return module_->getTrafficLightModuleState(); }
  auto decisionState() { return module_->decision_line_latch_.state(); }
  bool pendingDecisionStop() { return module_->decision_line_latch_.pendingStop(); }
  void setPeriod(double seconds) { module_->planner_param_.yellow_lamp_period=seconds; }
  void setStopMargin(double metres) { module_->planner_param_.stop_margin=metres; }
  void setYellowDeceleration(double value) { module_->planner_param_.signal_compliance_yellow_deceleration_mps2=value; }
  void setObsoleteTimed(bool value) {
    module_->planner_param_.signal_compliance_enable_timed_yellow=value;
    module_->planner_param_.signal_compliance_min_yellow_sec=0.;
    module_->planner_param_.signal_compliance_yellow_time_margin_sec=-100.;
  }
  void setFailOpen(bool enabled) { module_->planner_param_.competition_fail_open=enabled; }
  void setFarUnseen(bool enabled) { module_->planner_param_.defer_far_unseen_unknown=enabled; }
  void setUnseenUntilObserved(bool enabled) { module_->planner_param_.defer_unseen_until_observed=enabled; }
  double stoppingDistance()
  {
    return planning_utils::calcJudgeLineDistWithJerkLimit(data_->current_velocity->twist.linear.x,
      data_->current_acceleration->accel.accel.linear.x, data_->max_stop_acceleration_threshold,
      data_->max_stop_jerk_threshold,data_->delay_response_time);
  }
};

TEST_F(TrafficLightComplianceModuleTest, ABCDThreeFreshSourcesUseActualDirectionalSceneAndPath)
{
  for(const auto group:{201351,201365,201290,201326}) {
    SCOPED_TRACE(group); configure(group);
    tick(); pose(0.,0.); put(green());
    EXPECT_TRUE(modifiesToStop());
    EXPECT_TRUE(modifiesToStop());  // Same source, another planning callback.
    tick(); put(green()); EXPECT_TRUE(modifiesToStop());
    tick(); put(green()); EXPECT_FALSE(modifiesToStop());
    tick(); put(red()); EXPECT_TRUE(modifiesToStop());
  }
}

TEST_F(TrafficLightComplianceModuleTest, CalibratedDefaultsEnableDecisionLine)
{
  const TrafficLightModule::PlannerParam params{};
  EXPECT_TRUE(params.enable_yellow_passthrough);
  EXPECT_DOUBLE_EQ(params.signal_compliance_yellow_deceleration_mps2,6.);
  EXPECT_DOUBLE_EQ(params.signal_compliance_yellow_jerk_mps3,10.);
}

TEST_F(TrafficLightComplianceModuleTest, ABCDJerkDecisionBoundaryIgnoresOldLampPeriod)
{
  for(const auto group:{201351,201365,201290,201326}) {
    for(const double period:{2.,3.}) {
      for(const bool stoppable:{true,false}) {
        SCOPED_TRACE(group);
        SCOPED_TRACE(period);
        SCOPED_TRACE(stoppable);
        configure(group); pose(0.,5.);
        const double judge=*signal_compliance::jerkStoppingDistance(5.,8.,16.,data_->delay_response_time);
        const double target_distance=judge+(stoppable?.1:-.1);
        const double x=10.-3.845-target_distance;
        confirmGreenOnRealPath(x,5.); setPeriod(period);
        tick(); pose(x,5.); put(yellow());
        EXPECT_EQ(signalStop(),stoppable);
        EXPECT_FALSE(pass(target_distance)); // No second yellow-only override.
        EXPECT_EQ(modifiesToStop(),stoppable);
      }
    }
  }
}

TEST_F(TrafficLightComplianceModuleTest, StoppedYellowAndFirstMidYellowCannotCreateNewGoForAnyMission)
{
  for(const auto group:{201351,201365,201290,201326}) {
    configure(group); tick(); pose(6.,5.); put(yellow());
    EXPECT_TRUE(modifiesToStop()); EXPECT_FALSE(pass(.155));
    configure(group); tick(); pose(6.,0.); put(yellow());
    EXPECT_TRUE(modifiesToStop()); EXPECT_FALSE(pass(.155));
    for(int index=0;index<30;++index) {
      tick(); put(yellow()); EXPECT_TRUE(modifiesToStop());
    }
  }
}

TEST_F(TrafficLightComplianceModuleTest, BeforeLineRedMissingAndStaleCannotReuseGreenAuthorization)
{
  for(const auto group:{201351,201365,201290,201326}) {
    configure(group); setFailOpen(false); confirmGreenOnRealPath(6.,.2);
    tick(); put(red()); EXPECT_TRUE(modifiesToStop());
    confirmGreenOnRealPath(6.,.2);
    data_->traffic_light_id_map_raw_.clear(); tick(); EXPECT_TRUE(modifiesToStop());
    confirmGreenOnRealPath(6.,.2);
    tick(1.1); EXPECT_TRUE(modifiesToStop());
    confirmGreenOnRealPath(6.,.2);
    Signal unknown; unknown.elements={lamp(Element::UNKNOWN,Element::UNKNOWN)};
    tick(); put(unknown); EXPECT_TRUE(modifiesToStop());
  }
}

TEST_F(TrafficLightComplianceModuleTest, CompetitionDefaultMissingAndStaleGoWithoutNewLaunchFlags)
{
  for(const auto group:{201351,201365,201290,201326}) {
    configure(group,true,false); pose(0.,0.);
    EXPECT_FALSE(modifiesToStop());  // The traffic stack was never started.
    tick(); put(red()); EXPECT_TRUE(modifiesToStop());
    tick(1.1); EXPECT_FALSE(modifiesToStop());  // Existing timeout fail-open.
    data_->traffic_light_id_map_raw_.clear(); tick(); EXPECT_FALSE(modifiesToStop());
    Signal unknown; unknown.elements={lamp(Element::UNKNOWN,Element::UNKNOWN)};
    tick(); put(unknown); EXPECT_TRUE(modifiesToStop()); // Fresh unreadable != no publisher.
    confirmGreenOnRealPath(0.,0.); // Bypass did not retain an old green confirmation.
  }
}

TEST_F(TrafficLightComplianceModuleTest, FreshEmptyUnknownStreamStopsUntilThreeNewGreenSources)
{
  for(const auto group:{201351,201365,201290,201326}) {
    for(const bool explicit_unknown:{false,true}) {
      configure(group,true,false); pose(0.,0.);
      confirmGreenOnRealPath(0.,0.);
      Signal unknown;
      if(explicit_unknown) unknown.elements={lamp(Element::UNKNOWN,Element::UNKNOWN)};
      // The actual detector publishes an empty element list for UNKNOWN.
      // Fresh updates remain STOP; elapsed wall time is not a fail-open timer.
      for(int index=0;index<20;++index) {
        tick(); put(unknown); EXPECT_TRUE(modifiesToStop());
      }
      tick(); put(green()); EXPECT_TRUE(modifiesToStop());
      // Planner callbacks reusing one observation do not count as new frames.
      tick(.02); EXPECT_TRUE(modifiesToStop());
      tick(); put(green()); EXPECT_TRUE(modifiesToStop());
      tick(); put(green()); EXPECT_FALSE(modifiesToStop());
    }
  }
}

TEST_F(TrafficLightComplianceModuleTest, UnknownGraceRetainsOnlyConfirmedDirectionalGreenOnActualPath)
{
  for (const auto group : {201351,201365,201290,201326}) {
    for (const int count : {1,2}) {
      configure(group,true,false,10.,true,.2,count); pose(0.,2.);
      for (int n=0; n<count; ++n) {
        tick(.01); put(green()); EXPECT_EQ(modifiesToStop(), n<count-1);
      }
      Signal empty;
      tick(.05); put(empty); EXPECT_FALSE(modifiesToStop());
      // Neither another callback nor another UNKNOWN image starts a new lease.
      tick(.05); put(empty); EXPECT_FALSE(modifiesToStop());
      tick(.101); put(empty); EXPECT_TRUE(modifiesToStop());
      tick(.01); put(green()); EXPECT_EQ(modifiesToStop(), count>1);
    }
  }
}

TEST_F(TrafficLightComplianceModuleTest, UnknownGraceKnownColorMarkersAndRedYellowStopImmediately)
{
  for (const auto color : {Element::RED,Element::AMBER,Element::GREEN}) {
    configure(201351,true,false,10.,true,.2,1); pose(0.,2.);
    tick(.01); put(green()); EXPECT_FALSE(modifiesToStop());
    Signal empty;
    tick(.02); put(empty); EXPECT_FALSE(modifiesToStop());
    Signal evidence; auto marker=lamp(color,Element::UNKNOWN); marker.status=Element::UNKNOWN;
    evidence.elements={marker};
    tick(.02); put(evidence); EXPECT_TRUE(modifiesToStop());
    tick(.02); put(empty); EXPECT_TRUE(modifiesToStop());
  }
  for (const bool is_yellow : {false,true}) {
    configure(201351,true,false,10.,true,.2,1); pose(0.,2.);
    tick(.01); put(green()); EXPECT_FALSE(modifiesToStop());
    tick(.02); put(is_yellow ? yellow() : red()); EXPECT_TRUE(modifiesToStop());
  }
}

TEST_F(TrafficLightComplianceModuleTest, UnknownGraceCannotReleaseFromUnconfirmedOrWrongDirectionGreen)
{
  configure(201351,true,false,10.,true,.2,2); pose(0.,2.);
  tick(.01); put(green()); EXPECT_TRUE(modifiesToStop());
  Signal empty;
  tick(.02); put(empty); EXPECT_TRUE(modifiesToStop());
  configure(201326,true,false,10.,true,.2,1); pose(0.,2.);
  Signal circle_green; circle_green.elements={lamp(Element::GREEN)};
  tick(.01); put(circle_green); EXPECT_TRUE(modifiesToStop());
  tick(.02); put(empty); EXPECT_TRUE(modifiesToStop());
}

TEST_F(TrafficLightComplianceModuleTest, UnknownGraceOriginalGreenAgeAndMissingStillInvalidateLease)
{
  configure(201351,true,false,10.,true,.2,1); pose(0.,2.);
  tick(.01); put(green(),.15); EXPECT_FALSE(modifiesToStop());
  Signal empty;
  tick(.06); put(empty); EXPECT_TRUE(modifiesToStop());
  tick(.01); put(green()); EXPECT_FALSE(modifiesToStop());
  setFailOpen(false);
  data_->traffic_light_id_map_raw_.clear(); tick(.02); EXPECT_TRUE(modifiesToStop());
  tick(.02); put(empty); EXPECT_TRUE(modifiesToStop());
  tick(.01); put(green()); EXPECT_FALSE(modifiesToStop());
  tick(.02); put(empty,2.); EXPECT_TRUE(modifiesToStop());
  tick(.02); put(empty); EXPECT_TRUE(modifiesToStop());
}

TEST_F(TrafficLightComplianceModuleTest, FarUnseenEmptyDefersOnlyBeforeFirstReadableLamp)
{
  for (const auto group : {201351,201365,201290,201326}) {
    SCOPED_TRACE(group);
    Signal empty;
    configure(group,true,false); setFarUnseen(true);
    tick(); pose(-65.,5.); put(empty);
    EXPECT_FALSE(modifiesToStop(-100.));
    tick(); put(red()); EXPECT_TRUE(modifiesToStop(-100.));
    tick(); put(empty); EXPECT_TRUE(modifiesToStop(-100.));

    configure(group,true,false); setFarUnseen(true);
    tick(); pose(-65.,5.); put(green()); EXPECT_TRUE(modifiesToStop(-100.));
    tick(); put(green()); EXPECT_TRUE(modifiesToStop(-100.));
    tick(); put(green()); EXPECT_FALSE(modifiesToStop(-100.));
    tick(); put(empty); EXPECT_TRUE(modifiesToStop(-100.));

    configure(group,true,false); setFarUnseen(true);
    tick(); pose(-20.,5.); put(empty);
    EXPECT_TRUE(modifiesToStop(-100.));

    configure(group,true,false); setFarUnseen(false);
    tick(); pose(-65.,5.); put(empty);
    EXPECT_TRUE(modifiesToStop(-100.));
  }
}

TEST_F(TrafficLightComplianceModuleTest, FarUnseenDynamicBrakingBoundaryStaysClosed)
{
  configure(201351,true,false); setFarUnseen(true);
  Signal empty;
  tick(); pose(-55.,20.); put(empty);
  EXPECT_TRUE(modifiesToStop(-100.));
}

TEST_F(TrafficLightComplianceModuleTest, ABCDInitialUnseenRemainsOpenNearLineUntilReadableLamp)
{
  for (const auto group : {201351,201365,201290,201326}) {
    SCOPED_TRACE(group);
    configure(group,true,false); setUnseenUntilObserved(true);
    Signal empty;
    tick(); pose(-20.,5.); put(empty);
    EXPECT_FALSE(modifiesToStop(-100.));
    Signal explicit_unknown; explicit_unknown.elements={lamp(Element::UNKNOWN,Element::UNKNOWN)};
    tick(); put(explicit_unknown); EXPECT_FALSE(modifiesToStop(-100.));
    tick(); put(red()); EXPECT_TRUE(modifiesToStop(-100.));
    tick(); put(empty); EXPECT_TRUE(modifiesToStop(-100.));

    configure(group,true,false); setUnseenUntilObserved(true);
    tick(); pose(-20.,5.); put(empty); EXPECT_FALSE(modifiesToStop(-100.));
    tick(); put(green()); EXPECT_TRUE(modifiesToStop(-100.));
    tick(); put(green()); EXPECT_TRUE(modifiesToStop(-100.));
    tick(); put(green()); EXPECT_FALSE(modifiesToStop(-100.));
    tick(); put(empty); EXPECT_TRUE(modifiesToStop(-100.));

    configure(group,true,false); setUnseenUntilObserved(false);
    tick(); pose(-20.,5.); put(empty); EXPECT_TRUE(modifiesToStop(-100.));

    configure(group,true,false); setUnseenUntilObserved(true);
    Signal ambiguous; ambiguous.elements={lamp(Element::RED,Element::UNKNOWN)};
    tick(); pose(-20.,5.); put(ambiguous); EXPECT_TRUE(modifiesToStop(-100.));
    tick(); put(empty); EXPECT_TRUE(modifiesToStop(-100.));
  }
}

TEST_F(TrafficLightComplianceModuleTest, ShiftedCameraStopLineCannotClearBeforeOriginalMORAILine)
{
  for(const auto group:{201351,201365,201290,201326}) {
    configure(group,true,true,20.);
    tick(); pose(8.,2.); put(red());
    EXPECT_TRUE(modifiesToStop());
    EXPECT_EQ(state(),TrafficLightModule::State::APPROACH);
    // Neither the shifted nor original crossing silently revokes a selected
    // STOP. Only fresh route-direction authorization can release it.
    tick(); pose(17.1,2.); put(red());
    EXPECT_TRUE(modifiesToStop());
    EXPECT_EQ(state(),TrafficLightModule::State::APPROACH);
    confirmGreenOnRealPath(17.1,0.);
  }
}

TEST_F(TrafficLightComplianceModuleTest, YellowOffDoesNotPromoteOfficialCrossingStateIntoExistingDefault)
{
  configure(201351,true,false,20.);
  tick(); pose(8.,2.); put(red());
  EXPECT_FALSE(modifiesToStop());
  EXPECT_EQ(state(),TrafficLightModule::State::GO_OUT);
}

TEST_F(TrafficLightComplianceModuleTest, SourcePreservingQualifiedGapDoesNotInsertTransientStops)
{
  for(const bool qualified:{false,true}) {
    configure(201351,true,false);
    confirmGreenOnRealPath(0.,2.);
    int stop_callbacks=0;
    for(int frame=1;frame<=2;++frame) {
      tick();
      if(qualified) put(green(),frame*.1);
      else { Signal unknown; put(unknown); }
      stop_callbacks+=modifiesToStop();
    }
    for(int frame=0;frame<3;++frame) {
      tick(); put(green()); stop_callbacks+=modifiesToStop();
    }
    // Same permission inputs, except bounded qualified gaps keep the source.
    // Baseline has two UNKNOWN STOPs plus two re-confirmation STOPs.
    EXPECT_EQ(stop_callbacks,qualified ? 0 : 4);
    tick(); put(red()); EXPECT_TRUE(modifiesToStop());
  }
}

TEST_F(TrafficLightComplianceModuleTest, MissingOriginalLineDoesNotDisableLaneletYellowPolicy)
{
  configure(201351,true,true,20.,false);
  confirmGreenOnRealPath(6.,2.);
  tick(); pose(6.,2.); put(yellow());
  EXPECT_FALSE(modifiesToStop());
  EXPECT_FALSE(pass(.155));
}

TEST_F(TrafficLightComplianceModuleTest, ABCDCameraTargetUsesBumperDistanceOnceAndIgnoresOldTimedMode)
{
  data_->delay_response_time=.5;
  for (const auto group : {201351,201365,201290,201326}) {
    for (const bool timed : {false,true}) {
      // Target is3.155m away; original front-axle line14m away. stop4.0625m
      // exceeds only the camera target, and must now retain GO in both modes.
      configure(group,true,true,20.); setObsoleteTimed(timed);
      confirmGreenOnRealPath(3.,5.); tick(); put(yellow());
      EXPECT_FALSE(modifiesToStop());
      // No second3m wheelbase subtraction: target6.155m is stoppable.
      configure(group,true,true,20.); setObsoleteTimed(timed);
      confirmGreenOnRealPath(0.,5.); tick(); put(yellow());
      EXPECT_TRUE(modifiesToStop());
    }
  }
}

TEST_F(TrafficLightComplianceModuleTest, ABCDUnknownGraceCannotInventDecisionBoundaryGrant)
{
  data_->delay_response_time=.5;
  for (const auto group : {201351,201365,201290,201326}) {
    for (const bool expired : {false,true}) {
      configure(group,true,true,20.,true,.3,1);
      tick(); pose(0.,5.); put(green()); EXPECT_FALSE(modifiesToStop());
      Signal empty; tick(expired ? .31 : .05); put(empty);
      EXPECT_EQ(modifiesToStop(),expired);
      tick(.05); put(yellow()); EXPECT_TRUE(modifiesToStop());
    }
  }
}

TEST_F(TrafficLightComplianceModuleTest, ABCDPriorYellowStopDoesNotBecomeNewGoCloserToLine)
{
  data_->delay_response_time=.5;
  for (const auto group : {201351,201365,201290,201326}) {
    configure(group,true,true,20.); confirmGreenOnRealPath(0.,5.);
    tick(); put(yellow()); EXPECT_TRUE(modifiesToStop());
    tick(); pose(6.,5.); put(yellow()); EXPECT_TRUE(modifiesToStop());
    configure(group,true,true,20.); confirmGreenOnRealPath(6.5,2.);
    tick(); put(yellow()); EXPECT_FALSE(modifiesToStop());
    tick(); put(red()); EXPECT_FALSE(modifiesToStop()); // Committed GO retains later red.
    tick(); pose(6.5,0.); put(red()); // An actual stop ends that moving commitment.
    auto output=path(); ASSERT_TRUE(module_->modifyPathVelocity(&output));
    const auto stopped=std::find_if(output.points.begin(),output.points.end(),[](const auto & p) {
      return p.point.longitudinal_velocity_mps<.001;
    });
    ASSERT_NE(stopped,output.points.end());
    EXPECT_GE(stopped->point.pose.position.x,6.5-1e-6);
    EXPECT_EQ(state(),TrafficLightModule::State::APPROACH);
  }
}

TEST_F(TrafficLightComplianceModuleTest, SimpleYellowDecelerationIsIndependentOfGlobalBrakingAndAcceleration)
{
  data_->delay_response_time=.5;
  data_->max_stop_acceleration_threshold=-6.; data_->max_stop_jerk_threshold=-10.;
  configure(201351,true,true,20.); confirmGreenOnRealPath(3.,5.);
  data_->current_acceleration.reset(); // Simplified yellow does not dereference acceleration.
  tick(); put(yellow()); EXPECT_FALSE(modifiesToStop());
  EXPECT_DOUBLE_EQ(data_->max_stop_acceleration_threshold,-6.);
  EXPECT_DOUBLE_EQ(data_->max_stop_jerk_threshold,-10.);
  // Changing global deceleration does not change the dedicated b/J decision.
  configure(201351,true,true,20.); setYellowDeceleration(16.);
  confirmGreenOnRealPath(0.,5.);
  tick(); put(yellow()); EXPECT_TRUE(modifiesToStop());
}

TEST_F(TrafficLightComplianceModuleTest, AlreadyCrossedFrontAxleLineNeverAddsRedOrMissingStop)
{
  for(const auto group:{201351,201365,201290,201326}) {
    configure(group); pose(7.1,2.); data_->traffic_light_id_map_raw_.clear();
    EXPECT_FALSE(modifiesToStop()); EXPECT_EQ(state(),TrafficLightModule::State::GO_OUT);
    tick(); put(red()); EXPECT_FALSE(modifiesToStop());
    data_->traffic_light_id_map_raw_.clear(); tick(2.); EXPECT_FALSE(modifiesToStop());
    pose(6.6,2.); EXPECT_FALSE(modifiesToStop());  // <1m backward localization jitter.
    EXPECT_EQ(state(),TrafficLightModule::State::GO_OUT);
  }
}

TEST_F(TrafficLightComplianceModuleTest, YellowMasterOffBlocksEveryPreLineOverrideForABCD)
{
  for(const auto group:{201351,201365,201290,201326}) {
    for(const bool guarded:{true,false}) {
      SCOPED_TRACE(group);
      SCOPED_TRACE(guarded);
      configure(group,guarded,false);
      if(guarded) confirmGreenOnRealPath(6.,5.);
      else { tick(); pose(6.,5.); put(green()); EXPECT_FALSE(modifiesToStop()); }
      tick(); put(yellow());
      EXPECT_TRUE(signalStop());
      EXPECT_FALSE(pass(.155));
      EXPECT_TRUE(modifiesToStop());
    }
  }
}

TEST_F(TrafficLightComplianceModuleTest, DefaultYellowOnAndExplicitOffDoNotBlockAuthorizedGreen)
{
  EXPECT_TRUE(TrafficLightModule::PlannerParam{}.enable_yellow_passthrough);
  for(const auto group:{201351,201365,201290,201326}) {
    for (const bool master : {true,false}) {
      configure(group,true,master);
      confirmGreenOnRealPath(6.,0.);
    }
  }
}

TEST_F(TrafficLightComplianceModuleTest, FirstUnseenNegativeSpeedDoesNotCreatePersistentSignalStop)
{
  for (const auto group : {201351,201365,201290,201326}) {
    SCOPED_TRACE(group);
    configure(group,true,true,20.,true,.3,1);
    setUnseenUntilObserved(true);
    tick(); pose(-20.,.2); put(Signal{}); EXPECT_FALSE(modifiesToStop());
    // Recorded D sequence: another module stops the ego while D is still
    // unseen, then signed localization velocity briefly goes below -.1m/s.
    tick(); pose(-20.,-.236); put(Signal{}); EXPECT_FALSE(modifiesToStop());
    EXPECT_FALSE(pendingDecisionStop());
    tick(); pose(-20.,.275); put(Signal{}); EXPECT_FALSE(modifiesToStop());
    EXPECT_FALSE(pendingDecisionStop());
    // First real R/Y still stops; UNKNOWN or another negative sample must not
    // erase that established STOP history even after the shifted map line.
    tick(); pose(6.,.1); put(red()); EXPECT_TRUE(modifiesToStop());
    tick(); pose(7.1,-.236); put(Signal{}); EXPECT_TRUE(modifiesToStop());
    EXPECT_TRUE(pendingDecisionStop());
    tick(); pose(7.1,0.); put(Signal{}); EXPECT_TRUE(modifiesToStop());
    tick(); put(yellow()); EXPECT_TRUE(modifiesToStop());
    tick(); put(green()); EXPECT_FALSE(modifiesToStop());
  }
}

TEST_F(TrafficLightComplianceModuleTest, YellowOffPendingStopSurvivesFrontAxleOvershoot)
{
  for(const auto group:{201351,201365,201290,201326}) {
    configure(group,true,false); tick(); pose(6.3,0.); put(red());
    // Bumper is past the line, but front axle is still 0.7m BEFORE it.
    EXPECT_TRUE(modifiesToStop());
    pose(7.1,0.); put(red());
    EXPECT_TRUE(modifiesToStop());
    EXPECT_EQ(state(),TrafficLightModule::State::APPROACH);
    configure(group,true,false); pose(7.1,0.);
    data_->traffic_light_id_map_raw_.clear();
    EXPECT_FALSE(modifiesToStop());
    EXPECT_EQ(state(),TrafficLightModule::State::GO_OUT);
  }
}

TEST_F(TrafficLightComplianceModuleTest, LargeBackwardRelocationRearmsActualRedStopWithoutOneCycleGo)
{
  configure(201326); pose(7.1,2.); EXPECT_FALSE(modifiesToStop());
  EXPECT_EQ(state(),TrafficLightModule::State::GO_OUT);
  tick(); pose(5.,.2); put(red());
  EXPECT_TRUE(modifiesToStop());
  EXPECT_EQ(state(),TrafficLightModule::State::APPROACH);
}

TEST_F(TrafficLightComplianceModuleTest, PendingStopSurvivesUnknownAfterVirtualExitEvenWhenAlreadyStopped)
{
  for (const auto group:{201351,201365,201290,201326}) {
    configure(group,true,true,20.,true,.3,1);
    tick(); pose(6.,0.); put(red()); EXPECT_TRUE(modifiesToStop());
    tick(.4); pose(7.1,0.); put(Signal{}); EXPECT_TRUE(modifiesToStop());
    EXPECT_EQ(state(),TrafficLightModule::State::APPROACH);
    tick(); put(red()); EXPECT_TRUE(modifiesToStop());
    tick(); put(yellow()); EXPECT_TRUE(modifiesToStop()); // no yellow new departure
    tick(); put(Signal{}); EXPECT_TRUE(modifiesToStop());
  }
}

TEST_F(TrafficLightComplianceModuleTest, TerminalUnknownNeverMasksReadableEvidenceOrAllowsUnseenOrBoundary)
{
  configure(201351,true,true,20.,true,.3,1);
  tick(); pose(7.1,0.); put(Signal{}); EXPECT_TRUE(modifiesToStop());
  tick(); pose(6.,0.); put(red()); EXPECT_TRUE(modifiesToStop());
  tick(); pose(7.,0.); put(Signal{}); EXPECT_TRUE(modifiesToStop()); // equality
  tick(); pose(7.1,0.); put(Signal{}); EXPECT_TRUE(modifiesToStop());
  Signal ambiguous; ambiguous.elements={lamp(Element::RED,Element::UNKNOWN)};
  tick(); put(ambiguous); EXPECT_TRUE(modifiesToStop());
  tick(); put(yellow()); EXPECT_TRUE(modifiesToStop());
  setFailOpen(false);
  tick(); put(Signal{},1.1); EXPECT_TRUE(modifiesToStop());
}

TEST_F(TrafficLightComplianceModuleTest, CroppedVirtualLineRetainsGeometryButNeverMasksCurrentColors)
{
  for (const auto group:{201351,201365,201290,201326}) {
    SCOPED_TRACE(group);
    configure(group,true,true,20.,true,.3,1);
    tick(); pose(6.,0.); put(red()); EXPECT_TRUE(modifiesToStop());
    // Real virtual line x=10 is now behind the path start. Front x=15 has
    // passed it, but the official line x=20 remains ahead. The current-ego
    // synthetic stop target must not replace the real virtual-line distance.
    tick(.4); pose(12.,5.); put(Signal{}); EXPECT_TRUE(modifiesToStop(11.));
    EXPECT_EQ(state(),TrafficLightModule::State::APPROACH);
    tick(); put(red()); EXPECT_TRUE(modifiesToStop(11.));
    tick(); put(yellow()); EXPECT_TRUE(modifiesToStop(11.));
    Signal ambiguous; ambiguous.elements={lamp(Element::RED,Element::UNKNOWN)};
    tick(); put(ambiguous); EXPECT_TRUE(modifiesToStop(11.));
    tick(); put(Signal{}); EXPECT_TRUE(modifiesToStop(11.));
    // Independent geometry is not a latched grant after relocating backwards.
    tick(); pose(6.,0.); put(Signal{}); EXPECT_TRUE(modifiesToStop());
  }
}

TEST_F(TrafficLightComplianceModuleTest, CroppedPathCannotInventUnseenBoundaryOrStaleVirtualExit)
{
  for (const auto group:{201351,201365,201290,201326}) {
    configure(group,true,true,20.,true,.3,1);
    tick(); pose(12.,5.); put(Signal{}); EXPECT_TRUE(modifiesToStop(11.));
    tick(); pose(6.,0.); put(red()); EXPECT_TRUE(modifiesToStop());
    tick(); pose(7.,0.); put(Signal{}); EXPECT_TRUE(modifiesToStop(11.)); // front exactly x=10
    tick(); pose(12.,5.); put(Signal{}); EXPECT_TRUE(modifiesToStop(11.));
    setFailOpen(false);
    tick(); put(Signal{},1.1); EXPECT_TRUE(modifiesToStop(11.));
    tick(); pose(17.1,5.); put(red()); EXPECT_TRUE(modifiesToStop(11.));
    EXPECT_EQ(state(),TrafficLightModule::State::APPROACH);
  }
}

TEST_F(TrafficLightComplianceModuleTest, OfficialCrossingAndBackwardRelocationResetObservedApproach)
{
  configure(201351,true,true,20.,true,.3,1);
  tick(); pose(6.,0.); put(red()); EXPECT_TRUE(modifiesToStop());
  tick(); pose(7.1,0.); put(Signal{}); EXPECT_TRUE(modifiesToStop());
  tick(); pose(17.1,5.); put(green()); EXPECT_FALSE(modifiesToStop());
  tick(); pose(17.2,5.); put(green()); EXPECT_FALSE(modifiesToStop());
  EXPECT_EQ(state(),TrafficLightModule::State::GO_OUT);
  tick(); pose(7.1,0.); put(Signal{}); EXPECT_TRUE(modifiesToStop());
}

TEST_F(TrafficLightComplianceModuleTest, RegressedActualSourceAndClockNeedThreeNewGrants)
{
  configure(201326); confirmGreenOnRealPath();
  tick(); put(green(),.15); EXPECT_TRUE(signalStop());
  tick(); put(green()); EXPECT_TRUE(signalStop());
  tick(); put(green()); EXPECT_FALSE(signalStop());
  tick(-.05); put(green()); EXPECT_TRUE(signalStop());
  tick(); put(green()); EXPECT_TRUE(signalStop());
  tick(); put(green()); EXPECT_FALSE(signalStop());
}

TEST_F(TrafficLightComplianceModuleTest, GuardOffKeepsHistoricalMissingAndYellowGo)
{
  for(const auto group:{201351,201365,201290,201326}) {
    configure(group,false); pose(0.,0.);
    EXPECT_FALSE(modifiesToStop());
    tick(); put(yellow()); EXPECT_FALSE(modifiesToStop());
  }
}

TEST_F(TrafficLightComplianceModuleTest, ProtectedLeftNeverStartsFromStraightCircularGreen)
{
  configure(201326); pose(6.,0.);
  Signal wrong_direction; wrong_direction.elements={lamp(Element::GREEN)};
  for(int index=0;index<4;++index) {
    tick(); put(wrong_direction); EXPECT_TRUE(modifiesToStop());
  }
  tick(); pose(6.,5.); put(yellow()); EXPECT_TRUE(modifiesToStop());
  confirmGreenOnRealPath(6.,0.);
}

TEST_F(TrafficLightComplianceModuleTest, ABCDDedicatedJerkBoundaryDoesNotChangeCommonLegacyDistance)
{
  data_->max_stop_acceleration_threshold=-6.;
  data_->max_stop_jerk_threshold=-10.;
  data_->delay_response_time=.5;
  const double judge=*signal_compliance::jerkStoppingDistance(1.5,8.,16.,.5);
  for(const auto group:{201351,201365,201290,201326}) {
    for(const double period:{2.,3.}) {
      for(const double target_distance:{.8,1.}) {
        SCOPED_TRACE(group);
        SCOPED_TRACE(period);
        SCOPED_TRACE(target_distance);
        configure(group); const double x=10.-3.845-target_distance;
        confirmGreenOnRealPath(x,1.5); setPeriod(period);
        EXPECT_NEAR(stoppingDistance(),.671821954004133,1e-10);
        tick(); pose(x,1.5); put(yellow());
        EXPECT_EQ(signalStop(),target_distance>judge);
        EXPECT_FALSE(pass(target_distance));
        EXPECT_EQ(modifiesToStop(),target_distance>judge);
      }
    }
  }
}

TEST_F(TrafficLightComplianceModuleTest, ActualLowSpeedBoundaryHasNoDiscontinuousStopDecision)
{
  data_->max_stop_acceleration_threshold=-6.;
  data_->max_stop_jerk_threshold=-10.;
  data_->delay_response_time=.5;
  for(const double speed:{1.8-1e-5,1.8,1.8+1e-5}) {
    for(const double target_distance:{.9,1.3}) {
      SCOPED_TRACE(speed);
      SCOPED_TRACE(target_distance);
      configure(201326); const double x=10.-3.845-target_distance;
      confirmGreenOnRealPath(x,speed);
      tick(); pose(x,speed); put(yellow());
      const double judge=*signal_compliance::jerkStoppingDistance(speed,8.,16.,.5);
      EXPECT_EQ(modifiesToStop(),target_distance>judge);
    }
  }
}

TEST_F(TrafficLightComplianceModuleTest, GuardOffRetainsLegacyTriangularPassJudge)
{
  data_->max_stop_acceleration_threshold=-6.;
  data_->max_stop_jerk_threshold=-10.;
  data_->delay_response_time=.5;
  configure(201351,false); pose(0.,1.5);
  EXPECT_NEAR(stoppingDistance(),.671821954004133,1e-10);
  EXPECT_FALSE(pass(1.));  // Unchanged legacy distance says it can stop.
  EXPECT_TRUE(pass(.5));  // And still passes inside its old threshold.
}
TEST_F(TrafficLightComplianceModuleTest, StandardMarginAppliesToAllMissionsWithoutMovingCrossingLine)
{
  for (const auto group : {201351, 201365, 201290, 201326}) {
    SCOPED_TRACE(group);
    configure(group, true, false);
    setStopMargin(.75);
    tick(); pose(0., 0.); put(red());
    auto result=path();
    ASSERT_TRUE(module_->modifyPathVelocity(&result));
    const auto stopped=std::find_if(result.points.begin(),result.points.end(),[](const auto & p) {
      return p.point.longitudinal_velocity_mps < .001;
    });
    ASSERT_NE(stopped,result.points.end());
    EXPECT_NEAR(stopped->point.pose.position.x,10.-3.845-.75,1e-5);
    tick(); pose(6.99,0.);put(red());
    EXPECT_TRUE(modifiesToStop());
    EXPECT_EQ(state(),TrafficLightModule::State::APPROACH);
    tick();pose(7.01,0.);put(red());
    EXPECT_TRUE(modifiesToStop());
    EXPECT_EQ(state(),TrafficLightModule::State::APPROACH);
  }
}

TEST_F(TrafficLightComplianceModuleTest, ABCDJumpAcrossDecisionBoundaryCommitsOnlyDirectionalGreen)
{
  data_->delay_response_time=0.;
  for(const auto group:{201351,201365,201290,201326}) {
    configure(group,true,true,20.,true,.3,1);
    tick();pose(0.,5.);put(green());EXPECT_FALSE(modifiesToStop());
    EXPECT_EQ(decisionState(),signal_compliance::DecisionLineState::Approach);
    // Skip over equality between callbacks; first inside sample commits once.
    tick();pose(5.,5.);put(green());EXPECT_FALSE(modifiesToStop());
    EXPECT_EQ(decisionState(),signal_compliance::DecisionLineState::Go);
    tick();put(yellow());EXPECT_FALSE(modifiesToStop());
    tick();put(red());EXPECT_FALSE(modifiesToStop());
    tick();pose(5.,0.);put(red());EXPECT_TRUE(modifiesToStop());
    EXPECT_TRUE(pendingDecisionStop());
    tick();put(green());EXPECT_FALSE(modifiesToStop());
    EXPECT_EQ(decisionState(),signal_compliance::DecisionLineState::Approach);
  }
}

TEST_F(TrafficLightComplianceModuleTest, CroppedSyntheticZeroCannotCommitAndPendingStopRemainsForward)
{
  configure(201351,true,true,20.,true,0.,1);
  tick();pose(6.,0.);put(red());EXPECT_TRUE(modifiesToStop());
  tick();pose(12.,0.);put(red());
  auto output=path(11.);
  ASSERT_TRUE(module_->modifyPathVelocity(&output));
  const auto stop=std::find_if(output.points.begin(),output.points.end(),[](const auto & point) {
    return point.point.longitudinal_velocity_mps<.001;
  });
  ASSERT_NE(stop,output.points.end());
  EXPECT_GE(stop->point.pose.position.x,12.-1e-5);
  EXPECT_TRUE(pendingDecisionStop());
}
}  // namespace autoware::behavior_velocity_planner

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"traffic_light_compliance_module_gtest");
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
