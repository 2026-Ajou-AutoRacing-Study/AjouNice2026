// Copyright 2026 AjouNice2026 contributors
// Licensed under the Apache License, Version 2.0.
// Actual experimental Trajectory boundary; no simulator or vehicle-command output.
// The test runner must supply its isolated ROS master. Only this process uses setNow.
#include "../src/experimental/scene.hpp"
#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
class TrafficYellowSwitchExperimentalTest : public ::testing::Test
{
protected:
  using Element = autoware_perception_msgs::TrafficLightElement;
  using Signal = autoware_perception_msgs::TrafficLightGroup;
  ros::NodeHandle node_{"/traffic_yellow_experimental_test"};
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
    data_->max_stop_acceleration_threshold = -6.;
    data_->max_stop_jerk_threshold = -10.;
    data_->delay_response_time = .5;
    clock_ = std::make_shared<TrafficLightRos1Clock>();
    factors_ = std::make_shared<planning_factor_interface::PlanningFactorInterface>(
      &node_,"traffic_yellow_experimental_test");
  }

  void TearDown() override
  {
    module_.reset(); factors_.reset();
    node_.deleteParam("/traffic_yellow_experimental_test");
  }

  void configure(int64_t group, bool yellow_enabled, bool fail_open=true)
  {
    module_.reset(); group_ = group;
    lanelet::LineString3d line(lanelet::utils::getId(), {
      lanelet::Point3d(lanelet::utils::getId(),10.,-3.,0.),
      lanelet::Point3d(lanelet::utils::getId(),10.,3.,0.)});
    light_ = lanelet::TrafficLight::make(group,{}, {line});
    lanelet::Lanelet lane(group+1000);
    lane.attributes()["turn_direction"] = group==201326 ? "left" : "straight";
    TrafficLightModule::PlannerParam params{};
    params.tl_state_timeout = .5; params.yellow_lamp_period = 2.75;
    params.yellow_light_stop_velocity = 1.; params.stop_time_hysteresis = .1;
    params.enable_pass_judge = true; params.yellow_light_is_go = true;
    params.competition_fail_open = fail_open; params.enable_signal_compliance_guard = true;
    params.enable_yellow_passthrough = yellow_enabled;
    params.signal_compliance_front_axle_offset_m = 3.;
    params.use_official_judging_lines = true;
    params.signal_judging_lines = {{group,{{10.,-3.},{10.,3.}},{1.,0.}}};
    module_ = std::make_shared<TrafficLightModule>(group+1000,*light_,lane,line,
      group==201326,true,params,node_,"traffic_yellow_experimental_test",clock_,nullptr,factors_);
    module_->setRTCEnabled(false);
    data_->traffic_light_id_map_raw_.clear();
  }

  void tick(double seconds=.1)
  {
    now_ += seconds;
    ros::Time value; value.fromSec(now_); ros::Time::setNow(value);
  }

  void pose(double x, double speed)
  {
    nav_msgs::Odometry message;
    message.header.stamp = ros::Time::now(); message.header.frame_id = "map";
    message.pose.pose.position.x = x; message.pose.pose.orientation.w = 1.;
    message.twist.twist.linear.x = speed;
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
    if (group_==201326) result.elements={lamp(Element::RED),lamp(Element::GREEN,Element::LEFT_ARROW)};
    else if (group_==201290) result.elements={lamp(Element::GREEN),lamp(Element::GREEN,Element::LEFT_ARROW)};
    else result.elements={lamp(Element::GREEN)};
    return result;
  }

  Signal yellow()
  {
    Signal result; result.elements={lamp(Element::AMBER)};
    if (group_==201326) result.elements.push_back(lamp(Element::RED));
    return result;
  }

  Signal red() { Signal result; result.elements={lamp(Element::RED)}; return result; }

  void put(Signal signal, double age=0.)
  {
    signal.traffic_light_group_id=group_;
    TrafficSignalStamped stamped; stamped.stamp=ros::Time::now()-ros::Duration(age); stamped.signal=signal;
    data_->traffic_light_id_map_raw_[group_]=stamped;
  }

  bool modifiesToStop()
  {
    std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> points;
    for (double x=-30.;x<=40.;x+=.25) {
      autoware_internal_planning_msgs::PathPointWithLaneId point;
      point.point.pose.position.x=x; point.point.pose.orientation.w=1.;
      point.point.longitudinal_velocity_mps=8.; point.lane_ids={group_+1000};
      points.push_back(point);
    }
    auto built=Trajectory::Builder{}.build(points);
    EXPECT_TRUE(built);
    if (!built) return true;
    auto trajectory=*built;
    geometry_msgs::Point a,b,c,d;
    a.x=c.x=-30.; b.x=d.x=40.; a.y=b.y=3.; c.y=d.y=-3.;
    EXPECT_TRUE(module_->modifyPathVelocity(trajectory,{a,b},{c,d},*data_));
    const auto restored=trajectory.restore();
    return std::any_of(restored.begin(),restored.end(),[](const auto & point) {
      return point.point.longitudinal_velocity_mps<.001;
    });
  }

  void confirmGreen(double speed=1.5)
  {
    for (int index=0;index<3;++index) {
      tick(); pose(6.,speed); put(green());
      EXPECT_EQ(modifiesToStop(),index<2);
    }
  }
};

TEST_F(TrafficYellowSwitchExperimentalTest, ABCDMovingGreenToCloseYellowStopsOffAndPassesOn)
{
  // Front axle is 1m BEFORE the line; corrected triangular stop distance is
  // 1.2977m at 1.5m/s. This exercises the actual experimental trajectory path.
  for (const auto group : {201351,201365,201290,201326}) {
    for (const bool yellow_enabled : {false,true}) {
      SCOPED_TRACE(group);
      SCOPED_TRACE(yellow_enabled);
      configure(group,yellow_enabled); confirmGreen();
      tick(); pose(6.,1.5); put(yellow());
      EXPECT_EQ(modifiesToStop(),!yellow_enabled);
      EXPECT_EQ(module_->getTrafficLightModuleState(),TrafficLightModule::State::APPROACH);
    }
  }
}

TEST_F(TrafficYellowSwitchExperimentalTest, ABCDStoppedYellowCannotRestartEvenWithOptionOn)
{
  for (const auto group : {201351,201365,201290,201326}) {
    SCOPED_TRACE(group);
    configure(group,true); confirmGreen(0.);
    tick(); pose(6.,0.); put(yellow());
    EXPECT_TRUE(modifiesToStop());
    // Fresh module, first observation already yellow, is also not a grant.
    configure(group,true); tick(); pose(6.,0.); put(yellow());
    EXPECT_TRUE(modifiesToStop());
  }
}

TEST_F(TrafficYellowSwitchExperimentalTest, ABCDRedAndSignalLossStillStopWithOptionOn)
{
  for (const auto group : {201351,201365,201290,201326}) {
    SCOPED_TRACE(group);
    configure(group,true,false); confirmGreen();
    tick(); pose(6.,1.5); put(red()); EXPECT_TRUE(modifiesToStop());
    configure(group,true,false); confirmGreen();
    data_->traffic_light_id_map_raw_.clear(); tick(); pose(6.,1.5);
    EXPECT_TRUE(modifiesToStop());
  }
}

TEST_F(TrafficYellowSwitchExperimentalTest, CompetitionDefaultNoTrafficAndTimeoutGoButFreshRedStops)
{
  for (const auto group : {201351,201365,201290,201326}) {
    SCOPED_TRACE(group);
    configure(group,false); tick(); pose(0.,0.);
    EXPECT_FALSE(modifiesToStop());
    tick(); put(red()); EXPECT_TRUE(modifiesToStop());
    tick(); put(red(),.6); EXPECT_FALSE(modifiesToStop());
    data_->traffic_light_id_map_raw_.clear(); tick(); EXPECT_FALSE(modifiesToStop());
    Signal unknown; unknown.elements={lamp(Element::UNKNOWN,Element::UNKNOWN)};
    tick(); put(unknown); EXPECT_TRUE(modifiesToStop());
    configure(group,false,false); tick(); pose(0.,0.);
    EXPECT_TRUE(modifiesToStop()); // Existing fail-open=false remains available.
  }
}

TEST_F(TrafficYellowSwitchExperimentalTest, ABCDAlreadyCrossedStationaryAndFreshModuleClearWithOptionOff)
{
  EXPECT_FALSE(TrafficLightModule::PlannerParam{}.enable_yellow_passthrough);
  for (const auto group : {201351,201365,201290,201326}) {
    SCOPED_TRACE(group);
    configure(group,false); tick(); pose(6.3,0.); put(red());
    EXPECT_TRUE(modifiesToStop()); // Bumper crossed, front axle still 0.7m before line.
    tick(); pose(7.1,0.); put(red());
    EXPECT_FALSE(modifiesToStop());
    EXPECT_EQ(module_->getTrafficLightModuleState(),TrafficLightModule::State::GO_OUT);
    configure(group,false); tick(); pose(7.1,0.); // Fresh module, no signal.
    EXPECT_FALSE(modifiesToStop());
    EXPECT_EQ(module_->getTrafficLightModuleState(),TrafficLightModule::State::GO_OUT);
  }
}
}  // namespace autoware::behavior_velocity_planner::experimental

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"traffic_yellow_switch_experimental_gtest");
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
