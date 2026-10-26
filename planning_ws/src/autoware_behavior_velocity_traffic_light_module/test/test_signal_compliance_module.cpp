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

  void configure(int64_t group, bool guarded=true, bool yellow_passthrough=true)
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
    params.tl_state_timeout=1.; params.yellow_lamp_period=2.75;
    params.yellow_light_stop_velocity=1.; params.stop_time_hysteresis=.1;
    params.enable_pass_judge=true; params.yellow_light_is_go=true;
    // Existing dilemma/legacy tests are explicit ON comparisons, not defaults.
    params.enable_yellow_passthrough=yellow_passthrough;
    params.competition_fail_open=true; params.enable_signal_compliance_guard=guarded;
    params.signal_compliance_front_axle_offset_m=3.;
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
  Path path()
  {
    Path result;
    result.header.frame_id="map"; result.header.stamp=ros::Time::now();
    for(double x=-30.;x<=40.;x+=.25) {
      autoware_internal_planning_msgs::PathPointWithLaneId point;
      point.point.pose.position.x=x; point.point.pose.orientation.w=1.;
      point.point.longitudinal_velocity_mps=8.; point.lane_ids={group_+1000};
      result.points.push_back(point);
    }
    geometry_msgs::Point a,b,c,d;
    a.x=c.x=-30.; b.x=d.x=40.; a.y=b.y=3.; c.y=d.y=-3.;
    result.left_bound={a,b}; result.right_bound={c,d};
    return result;
  }
  bool modifiesToStop()
  {
    auto result=path();
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
  bool signalStop() { return module_->isStopSignal(); }
  bool pass(double bumper_target_distance) { return module_->isPassthrough(bumper_target_distance); }
  auto state() { return module_->getTrafficLightModuleState(); }
  void setPeriod(double seconds) { module_->planner_param_.yellow_lamp_period=seconds; }
  void setStopMargin(double metres) { module_->planner_param_.stop_margin=metres; }
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

TEST_F(TrafficLightComplianceModuleTest, ABCDActualJerkStoppingBoundaryDoesNotUseTwoOrThreeSecondPeriod)
{
  for(const auto group:{201351,201365,201290,201326}) {
    for(const double period:{2.,3.}) {
      for(const bool stoppable:{true,false}) {
        SCOPED_TRACE(group);
        SCOPED_TRACE(period);
        SCOPED_TRACE(stoppable);
        configure(group); pose(0.,5.);
        const double judge=stoppingDistance(); ASSERT_GT(judge,1.);
        const double front_distance=judge+(stoppable?.1:-.1);
        const double x=10.-3.-front_distance;
        confirmGreenOnRealPath(x,5.); setPeriod(period);
        tick(); pose(x,5.); put(yellow());
        EXPECT_TRUE(signalStop());
        EXPECT_EQ(pass(front_distance-.845),!stoppable);
        EXPECT_EQ(modifiesToStop(),stoppable);
      }
    }
  }
}

TEST_F(TrafficLightComplianceModuleTest, StoppedYellowAndFirstMidYellowCannotCreateNewGoForAnyMission)
{
  for(const auto group:{201351,201365,201290,201326}) {
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
    configure(group); confirmGreenOnRealPath(6.,.2);
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

TEST_F(TrafficLightComplianceModuleTest, DefaultYellowMasterOffDoesNotBlockAuthorizedGreen)
{
  EXPECT_FALSE(TrafficLightModule::PlannerParam{}.enable_yellow_passthrough);
  for(const auto group:{201351,201365,201290,201326}) {
    configure(group,true,false);
    confirmGreenOnRealPath(6.,0.);
  }
}

TEST_F(TrafficLightComplianceModuleTest, StoppedPastFrontAxleLineClearsEvenAfterFreshModuleWithYellowOff)
{
  for(const auto group:{201351,201365,201290,201326}) {
    configure(group,true,false); tick(); pose(6.3,0.); put(red());
    // Bumper is past the line, but front axle is still 0.7m BEFORE it.
    EXPECT_TRUE(modifiesToStop());
    pose(7.1,0.); put(red());
    EXPECT_FALSE(modifiesToStop());
    EXPECT_EQ(state(),TrafficLightModule::State::GO_OUT);
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
  confirmGreenOnRealPath(6.,0.);
}

TEST_F(TrafficLightComplianceModuleTest, ABCDTriangularBrakingUsesPositiveStopTimeOnActualPath)
{
  data_->max_stop_acceleration_threshold=-6.;
  data_->max_stop_jerk_threshold=-10.;
  data_->delay_response_time=.5;
  // Independently integrated stopping distance is 1.297722558m; the unchanged
  // common function's negative-time triangular branch returns .671821954m.
  for(const auto group:{201351,201365,201290,201326}) {
    for(const double period:{2.,3.}) {
      for(const double front_distance:{1.,1.4}) {
        SCOPED_TRACE(group);
        SCOPED_TRACE(period);
        SCOPED_TRACE(front_distance);
        configure(group); const double x=7.-front_distance;
        confirmGreenOnRealPath(x,1.5); setPeriod(period);
        EXPECT_NEAR(stoppingDistance(),.671821954004133,1e-10);
        tick(); pose(x,1.5); put(yellow());
        EXPECT_TRUE(signalStop());
        EXPECT_EQ(pass(front_distance-.845),front_distance<1.297722558);
        EXPECT_EQ(modifiesToStop(),front_distance>1.297722558);
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
    for(const double front_distance:{1.2,1.7}) {
      SCOPED_TRACE(speed);
      SCOPED_TRACE(front_distance);
      configure(201326); const double x=7.-front_distance;
      confirmGreenOnRealPath(x,speed);
      tick(); pose(x,speed); put(yellow());
      EXPECT_EQ(modifiesToStop(),front_distance>1.62);
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
    EXPECT_FALSE(modifiesToStop());
    EXPECT_EQ(state(),TrafficLightModule::State::GO_OUT);
  }
}
}  // namespace autoware::behavior_velocity_planner

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"traffic_light_compliance_module_gtest");
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
