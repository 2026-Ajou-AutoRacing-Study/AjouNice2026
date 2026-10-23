// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "../src/experimental/scene.hpp"
#include "../src/scene.hpp"

#include <autoware/behavior_velocity_traffic_light_module/ros1_support.hpp>
#include <autoware/behavior_velocity_planner_common/planner_data.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{

class TrafficLightModuleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Setup common objects
    clock_ = std::make_shared<TrafficLightRos1Clock>();
    logger_ = "test_logger";
    time_keeper_ = nullptr;
    planning_factor_interface_ = nullptr;

    // Default params
    planner_param_.stop_margin = 0.0;
    planner_param_.tl_state_timeout = 1.0;
    planner_param_.yellow_lamp_period = 3.0;
    planner_param_.yellow_light_stop_velocity = 1.0;
    planner_param_.stop_time_hysteresis = 0.0;
    planner_param_.enable_pass_judge = false;
    planner_param_.enable_arrow_aware_yellow_passing = true;  // Key param for this test
    planner_param_.max_behind_dist_to_stop_for_restart_suppression = 0.0;
    planner_param_.min_behind_dist_to_stop_for_restart_suppression = 0.0;
    planner_param_.v2i_use_remaining_time = false;
    planner_param_.v2i_last_time_allowed_to_pass = 0.0;
    planner_param_.v2i_velocity_threshold = 0.0;
    planner_param_.v2i_required_time_to_departure = 0.0;

    // Dummy lanelet data
    lanelet::Points3d points;
    points.push_back(lanelet::Point3d(lanelet::utils::getId(), 0, 0, 0));
    points.push_back(lanelet::Point3d(lanelet::utils::getId(), 1, 0, 0));
    lanelet::LineString3d ls(lanelet::utils::getId(), points);

    traffic_light_ptr_ = lanelet::TrafficLight::make(12345, {}, {ls});
    const auto & traffic_light_reg_elem = *traffic_light_ptr_;
    lanelet::Lanelet lane(100);
    lanelet::LineString3d stop_line(200);

    // Create module
    module_ = std::make_shared<TrafficLightModule>(
      100, traffic_light_reg_elem, lane, stop_line, true /* is_turn_lane */,
      true /* has_static_arrow */, planner_param_, node_, logger_, clock_, time_keeper_,
      planning_factor_interface_);

    // Setup planner data
    auto node = get_test_node();
    planner_data_ = std::make_shared<PlannerData>(*node);
    planner_data_->current_odometry = std::make_shared<geometry_msgs::PoseStamped>();
    planner_data_->current_velocity = std::make_shared<geometry_msgs::TwistStamped>();
    planner_data_->current_acceleration =
      std::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
    planner_data_->is_simulation = false;
    setPlannerData(planner_data_);
  }

  std::shared_ptr<ros::NodeHandle> get_test_node()
  {
    auto node = std::make_shared<ros::NodeHandle>("/full_traffic_light_test");
    node->setParam("wheel_radius", 0.3);
    node->setParam("wheel_width", 0.2);
    node->setParam("wheel_base", 2.7);
    node->setParam("wheel_tread", 1.5);
    node->setParam("front_overhang", 1.0);
    node->setParam("rear_overhang", 1.0);
    node->setParam("left_overhang", 0.5);
    node->setParam("right_overhang", 0.5);
    node->setParam("vehicle_height", 1.5);
    node->setParam("max_steer_angle", 0.5);
    node->setParam("max_accel", 1.0);
    node->setParam("min_accel", -1.0);
    node->setParam("max_jerk", 1.0);
    node->setParam("min_jerk", -1.0);
    node->setParam("system_delay", 0.0);
    node->setParam("delay_response_time", 0.0);
    node->setParam("max_stop_acceleration_threshold", 1.0);
    node->setParam("max_stop_jerk_threshold", 1.0);
    return node;
  }

  std::shared_ptr<TrafficLightModule> module_;
  std::shared_ptr<PlannerData> planner_data_;
  TrafficLightModule::PlannerParam planner_param_;
  std::string logger_{"test"};
  ros::NodeHandle node_{"/full_traffic_light_test"};
  std::shared_ptr<TrafficLightRos1Clock> clock_;
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;

  std::shared_ptr<planning_factor_interface::PlanningFactorInterface> planning_factor_interface_;
  lanelet::TrafficLight::Ptr traffic_light_ptr_;

  // Helper to set traffic signal
  void setTrafficSignal(const autoware_perception_msgs::TrafficLightGroup & signal)
  {
    TrafficSignalStamped signal_stamped;
    signal_stamped.stamp = clock_->now();
    signal_stamped.signal = signal;
    planner_data_->traffic_light_id_map_raw_[12345] = signal_stamped;
  }

  autoware_perception_msgs::TrafficLightElement createElement(
    uint8_t color, uint8_t shape, uint8_t status)
  {
    autoware_perception_msgs::TrafficLightElement element;
    element.color = color;
    element.shape = shape;
    element.status = status;
    element.confidence = 1.0;
    return element;
  }

  using YellowState = TrafficLightModule::YellowState;

  YellowState getYellowTransitionState() { return module_->yellow_transition_state_; }

  void setPlannerData(const std::shared_ptr<PlannerData> & planner_data)
  {
    module_->setPlannerData(planner_data);
  }

  bool callIsStopSignal() { return module_->isStopSignal(); }

  void setCompetitionFailOpen(const bool enabled)
  {
    module_->planner_param_.competition_fail_open = enabled;
  }

  void verifyTransition(
    const autoware_perception_msgs::TrafficLightGroup & first_signal,
    const autoware_perception_msgs::TrafficLightGroup & second_signal,
    YellowState expected_state, bool expected_stop)
  {
    setTrafficSignal(first_signal);
    callIsStopSignal();

    setTrafficSignal(second_signal);
    bool stop = callIsStopSignal();

    EXPECT_EQ(getYellowTransitionState(), expected_state);
    if (expected_stop) {
      EXPECT_TRUE(stop);
    } else {
      EXPECT_FALSE(stop);
    }
  }
};

TEST_F(TrafficLightModuleTest, TransitionToYellowFromGreen)
{
  using Element = autoware_perception_msgs::TrafficLightElement;

  // 1. Set Green state
  autoware_perception_msgs::TrafficLightGroup green_signal;
  green_signal.elements.push_back(
    createElement(Element::GREEN, Element::CIRCLE, Element::SOLID_ON));

  // 2. Transition to Yellow
  autoware_perception_msgs::TrafficLightGroup yellow_signal;
  yellow_signal.elements.push_back(
    createElement(Element::AMBER, Element::CIRCLE, Element::SOLID_ON));

  // Should PASS (return false for stop signal) because it's a turn lane with static arrow and came
  // from Green
  verifyTransition(green_signal, yellow_signal, YellowState::kFromGreen, false);
}

TEST_F(TrafficLightModuleTest, TransitionToYellowFromRed)
{
  using Element = autoware_perception_msgs::TrafficLightElement;

  // 1. Set Red state (or Red Arrow)
  autoware_perception_msgs::TrafficLightGroup red_signal;
  red_signal.elements.push_back(createElement(Element::RED, Element::CIRCLE, Element::SOLID_ON));

  // 2. Transition to Yellow
  autoware_perception_msgs::TrafficLightGroup yellow_signal;
  yellow_signal.elements.push_back(
    createElement(Element::AMBER, Element::CIRCLE, Element::SOLID_ON));

  // Should STOP (return true for stop signal) because it did NOT come from Green
  verifyTransition(red_signal, yellow_signal, YellowState::kFromNonGreen, true);
}

TEST_F(TrafficLightModuleTest, StateResetWhenYellowEnds)
{
  using Element = autoware_perception_msgs::TrafficLightElement;

  // 1. Establish Yellow From Green state
  autoware_perception_msgs::TrafficLightGroup green_signal;
  green_signal.elements.push_back(
    createElement(Element::GREEN, Element::CIRCLE, Element::SOLID_ON));
  setTrafficSignal(green_signal);
  callIsStopSignal();

  autoware_perception_msgs::TrafficLightGroup yellow_signal;
  yellow_signal.elements.push_back(
    createElement(Element::AMBER, Element::CIRCLE, Element::SOLID_ON));
  setTrafficSignal(yellow_signal);
  callIsStopSignal();

  EXPECT_EQ(getYellowTransitionState(), YellowState::kFromGreen);

  // 2. Transition to Red
  autoware_perception_msgs::TrafficLightGroup red_signal;
  red_signal.elements.push_back(createElement(Element::RED, Element::CIRCLE, Element::SOLID_ON));
  setTrafficSignal(red_signal);

  callIsStopSignal();

  // Verify state reset
  EXPECT_EQ(getYellowTransitionState(), YellowState::kNotYellow);
}

TEST_F(TrafficLightModuleTest, NotTurnLane)
{
  using Element = autoware_perception_msgs::TrafficLightElement;

  // Re-create module with is_turn_lane = false
  lanelet::Points3d points;
  points.push_back(lanelet::Point3d(lanelet::utils::getId(), 0, 0, 0));
  points.push_back(lanelet::Point3d(lanelet::utils::getId(), 1, 0, 0));
  lanelet::LineString3d ls(lanelet::utils::getId(), points);

  auto traffic_light_ptr = lanelet::TrafficLight::make(12345, {}, {ls});
  const auto & traffic_light_reg_elem = *traffic_light_ptr;
  lanelet::Lanelet lane(100);
  lanelet::LineString3d stop_line(200);
  module_ = std::make_shared<TrafficLightModule>(
    100, traffic_light_reg_elem, lane, stop_line, false /* is_turn_lane */,
    true /* has_static_arrow */, planner_param_, node_, logger_, clock_, time_keeper_,
    planning_factor_interface_);

  auto node = get_test_node();

  planner_data_ = std::make_shared<PlannerData>(*node);
  planner_data_->current_odometry = std::make_shared<geometry_msgs::PoseStamped>();
  planner_data_->current_velocity = std::make_shared<geometry_msgs::TwistStamped>();
  planner_data_->current_acceleration =
    std::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
  planner_data_->is_simulation = false;
  setPlannerData(planner_data_);

  // 1. Green -> Yellow
  autoware_perception_msgs::TrafficLightGroup green_signal;
  green_signal.elements.push_back(
    createElement(Element::GREEN, Element::CIRCLE, Element::SOLID_ON));

  autoware_perception_msgs::TrafficLightGroup yellow_signal;
  yellow_signal.elements.push_back(
    createElement(Element::AMBER, Element::CIRCLE, Element::SOLID_ON));

  // State is tracked (kFromGreen) but should STOP because it is not a turn lane
  verifyTransition(green_signal, yellow_signal, YellowState::kFromGreen, true);
}

TEST_F(TrafficLightModuleTest, NoStaticArrow)
{
  using Element = autoware_perception_msgs::TrafficLightElement;

  // Re-create module with has_static_arrow = false
  lanelet::Points3d points;
  points.push_back(lanelet::Point3d(lanelet::utils::getId(), 0, 0, 0));
  points.push_back(lanelet::Point3d(lanelet::utils::getId(), 1, 0, 0));
  lanelet::LineString3d ls(lanelet::utils::getId(), points);

  auto traffic_light_ptr = lanelet::TrafficLight::make(12345, {}, {ls});
  const auto & traffic_light_reg_elem = *traffic_light_ptr;
  lanelet::Lanelet lane(100);
  lanelet::LineString3d stop_line(200);
  module_ = std::make_shared<TrafficLightModule>(
    100, traffic_light_reg_elem, lane, stop_line, true /* is_turn_lane */,
    false /* has_static_arrow */, planner_param_, node_, logger_, clock_, time_keeper_,
    planning_factor_interface_);

  auto node = get_test_node();
  planner_data_ = std::make_shared<PlannerData>(*node);
  planner_data_->current_odometry = std::make_shared<geometry_msgs::PoseStamped>();
  planner_data_->current_velocity = std::make_shared<geometry_msgs::TwistStamped>();
  planner_data_->current_acceleration =
    std::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
  planner_data_->is_simulation = false;
  setPlannerData(planner_data_);

  // 1. Green -> Yellow
  autoware_perception_msgs::TrafficLightGroup green_signal;
  green_signal.elements.push_back(
    createElement(Element::GREEN, Element::CIRCLE, Element::SOLID_ON));

  autoware_perception_msgs::TrafficLightGroup yellow_signal;
  yellow_signal.elements.push_back(
    createElement(Element::AMBER, Element::CIRCLE, Element::SOLID_ON));

  // State is tracked (kFromGreen) but should STOP because there is no static arrow
  verifyTransition(green_signal, yellow_signal, YellowState::kFromGreen, true);
}

TEST_F(TrafficLightModuleTest, UnknownToYellow)
{
  using Element = autoware_perception_msgs::TrafficLightElement;

  // 1. Directly set Yellow state (No previous Green)
  autoware_perception_msgs::TrafficLightGroup yellow_signal;
  yellow_signal.elements.push_back(
    createElement(Element::AMBER, Element::CIRCLE, Element::SOLID_ON));
  setTrafficSignal(yellow_signal);

  // Call isStopSignal
  bool stop = callIsStopSignal();

  // Verify internal state remains kNotYellow (default safe state)
  EXPECT_EQ(getYellowTransitionState(), YellowState::kNotYellow);

  // Should STOP because the origin of yellow is unknown (assumed unsafe)
  EXPECT_TRUE(stop);
}

TEST_F(TrafficLightModuleTest, FeatureDisabled)
{
  using Element = autoware_perception_msgs::TrafficLightElement;

  // Re-create module with enable_arrow_aware_yellow_passing = false
  lanelet::Points3d points;
  points.push_back(lanelet::Point3d(lanelet::utils::getId(), 0, 0, 0));
  points.push_back(lanelet::Point3d(lanelet::utils::getId(), 1, 0, 0));
  lanelet::LineString3d ls(lanelet::utils::getId(), points);

  auto traffic_light_ptr = lanelet::TrafficLight::make(12345, {}, {ls});
  const auto & traffic_light_reg_elem = *traffic_light_ptr;
  lanelet::Lanelet lane(100);
  lanelet::LineString3d stop_line(200);

  auto disabled_param = planner_param_;
  disabled_param.enable_arrow_aware_yellow_passing = false;  // Disable feature

  module_ = std::make_shared<TrafficLightModule>(
    100, traffic_light_reg_elem, lane, stop_line, true /* is_turn_lane */,
    true /* has_static_arrow */, disabled_param, node_, logger_, clock_, time_keeper_,
    planning_factor_interface_);

  auto node = get_test_node();
  planner_data_ = std::make_shared<PlannerData>(*node);
  planner_data_->current_odometry = std::make_shared<geometry_msgs::PoseStamped>();
  planner_data_->current_velocity = std::make_shared<geometry_msgs::TwistStamped>();
  planner_data_->current_acceleration =
    std::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
  planner_data_->is_simulation = false;
  setPlannerData(planner_data_);

  // 1. Green -> Yellow
  autoware_perception_msgs::TrafficLightGroup green_signal;
  green_signal.elements.push_back(
    createElement(Element::GREEN, Element::CIRCLE, Element::SOLID_ON));
  setTrafficSignal(green_signal);
  callIsStopSignal();

  autoware_perception_msgs::TrafficLightGroup yellow_signal;
  yellow_signal.elements.push_back(
    createElement(Element::AMBER, Element::CIRCLE, Element::SOLID_ON));
  setTrafficSignal(yellow_signal);

  bool stop = callIsStopSignal();

  // State is NOT verified to be tracked because logic is skipped when disabled
  // Internal state should remain kNotYellow
  EXPECT_EQ(getYellowTransitionState(), YellowState::kNotYellow);

  // But should STOP because the feature is disabled
  EXPECT_TRUE(stop);
}

TEST_F(TrafficLightModuleTest, CompetitionFailOpenOnMissingSignal)
{
  EXPECT_TRUE(callIsStopSignal());
  setCompetitionFailOpen(true);
  EXPECT_FALSE(callIsStopSignal());
}

TEST_F(TrafficLightModuleTest, CompetitionFailOpenOnUnknownSignal)
{
  using Element = autoware_perception_msgs::TrafficLightElement;
  autoware_perception_msgs::TrafficLightGroup unknown_signal;
  unknown_signal.elements.push_back(
    createElement(Element::UNKNOWN, Element::UNKNOWN, Element::UNKNOWN));
  setTrafficSignal(unknown_signal);

  EXPECT_TRUE(callIsStopSignal());
  setCompetitionFailOpen(true);
  EXPECT_FALSE(callIsStopSignal());
}

}  // namespace autoware::behavior_velocity_planner

// Production BVP runs the experimental plugin (behavior_planning.launch,
// bvp_experimental:=true). The cases below pin its decision for the planner
// inputs that traffic_signal_profiles.yaml can deliver per competition section.
namespace autoware::behavior_velocity_planner::experimental
{
using Element = autoware_perception_msgs::TrafficLightElement;

enum class Lamp { RC, YC, GC, GLA };

class TrafficLightModuleTest : public ::testing::Test
{
protected:
  static constexpr lanelet::Id kRegId = 201326;

  void SetUp() override
  {
    ros::NodeHandle node("/experimental_traffic_light_test");
    node.setParam("wheel_radius", 0.3);
    node.setParam("wheel_width", 0.2);
    node.setParam("wheel_base", 2.7);
    node.setParam("wheel_tread", 1.5);
    node.setParam("front_overhang", 1.0);
    node.setParam("rear_overhang", 1.0);
    node.setParam("left_overhang", 0.5);
    node.setParam("right_overhang", 0.5);
    node.setParam("vehicle_height", 1.5);
    node.setParam("max_steer_angle", 0.5);
    // behavior_velocity_modules.yaml
    node.setParam("max_accel", -6.0);
    node.setParam("max_jerk", -10.0);
    node.setParam("delay_response_time", 0.5);
    planner_data_ = std::make_unique<PlannerData>(node);
    planner_data_->current_odometry = std::make_shared<geometry_msgs::PoseStamped>();
    planner_data_->current_velocity = std::make_shared<geometry_msgs::TwistStamped>();
    planner_data_->current_acceleration =
      std::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
    planner_data_->is_simulation = false;

    // traffic_light section of behavior_velocity_modules.yaml
    param_.stop_margin = 0.0;
    param_.tl_state_timeout = 1.0;
    param_.stop_time_hysteresis = 0.1;
    param_.yellow_lamp_period = 2.75;
    param_.yellow_light_stop_velocity = 1.0;
    param_.enable_pass_judge = true;
    param_.yellow_light_is_go = true;
    param_.competition_fail_open = true;
    param_.enable_arrow_aware_yellow_passing = false;
    param_.min_behind_dist_to_stop_for_restart_suppression = 0.5;
    param_.max_behind_dist_to_stop_for_restart_suppression = 1.0;
    param_.v2i_use_remaining_time = false;
    param_.v2i_last_time_allowed_to_pass = 2.0;
    param_.v2i_velocity_threshold = 0.5;
    param_.v2i_required_time_to_departure = 3.0;
  }

  // turn_direction of the decision lane: "straight" for A/B/C, "left" for D-2.
  void makeModule(const std::string & turn_direction)
  {
    lanelet::Points3d points{
      lanelet::Point3d(lanelet::utils::getId(), 0, 0, 0),
      lanelet::Point3d(lanelet::utils::getId(), 1, 0, 0)};
    lanelet::LineString3d light(lanelet::utils::getId(), points);
    reg_elem_ = lanelet::TrafficLight::make(kRegId, {}, {light});
    lanelet::Lanelet lane(lanelet::utils::getId());
    lane.setAttribute("turn_direction", turn_direction);
    module_ = std::make_shared<TrafficLightModule>(
      lane.id(), *reg_elem_, lane, lanelet::LineString3d(lanelet::utils::getId()),
      turn_direction == "left" || turn_direction == "right", false, param_, node_, "test",
      std::make_shared<TrafficLightRos1Clock>(), nullptr, nullptr);
  }

  static Element element(const uint8_t color, const uint8_t shape)
  {
    Element e;
    e.color = color;
    e.shape = shape;
    e.status = Element::SOLID_ON;
    e.confidence = 1.0;
    return e;
  }

  // Mirrors traffic_light_node.py: one group per message, map cleared per message.
  void publish(const std::vector<Element> & elements, const lanelet::Id id = kRegId, double age = 0.0)
  {
    TrafficSignalStamped signal;
    signal.stamp = ros::Time::now() - ros::Duration(age);
    signal.signal.traffic_light_group_id = id;
    signal.signal.elements = elements;
    planner_data_->traffic_light_id_map_raw_.clear();
    planner_data_->traffic_light_id_map_raw_[id] = signal;
  }

  void publish(const std::vector<Lamp> & lamps)
  {
    std::vector<Element> elements;
    for (const auto lamp : lamps) {
      switch (lamp) {
        case Lamp::RC: elements.push_back(element(Element::RED, Element::CIRCLE)); break;
        case Lamp::YC: elements.push_back(element(Element::AMBER, Element::CIRCLE)); break;
        case Lamp::GC: elements.push_back(element(Element::GREEN, Element::CIRCLE)); break;
        case Lamp::GLA: elements.push_back(element(Element::GREEN, Element::LEFT_ARROW)); break;
      }
    }
    publish(elements);
  }

  bool isStop() { return module_->isStopSignal(*planner_data_); }

  bool passthrough(const double distance, const double velocity)
  {
    auto twist = std::make_shared<geometry_msgs::TwistStamped>();
    twist->twist.linear.x = velocity;
    planner_data_->current_velocity = twist;
    return module_->isPassthrough(distance, *planner_data_);
  }

  double judgeLine(const double velocity) const
  {
    return planning_utils::calcJudgeLineDistWithJerkLimit(
      velocity, 0.0, planner_data_->max_stop_acceleration_threshold,
      planner_data_->max_stop_jerk_threshold, planner_data_->delay_response_time);
  }

  ros::NodeHandle node_{"/experimental_traffic_light_test"};
  std::unique_ptr<PlannerData> planner_data_;
  TrafficLightModule::PlannerParam param_;
  lanelet::TrafficLight::Ptr reg_elem_;
  std::shared_ptr<TrafficLightModule> module_;
};

// gtest needs a suite name distinct from the original-module fixture above.
class ExperimentalTrafficLightTest : public TrafficLightModuleTest
{
};

struct PhaseCase
{
  std::string section;
  std::string turn_direction;
  std::vector<Lamp> lamps;
  bool stop_with_yellow_go;     // yellow_light_is_go: true (current)
  bool stop_without_yellow_go;  // yellow_light_is_go: false
};

TEST_F(ExperimentalTrafficLightTest, SectionPhaseDecisions)
{
  using L = Lamp;
  const std::vector<PhaseCase> cases{
    // A/B simple_3color, C straight_left_combined; route goes straight at all three.
    {"A/B/C", "straight", {L::RC}, true, true},
    {"A/B/C", "straight", {L::YC}, false, true},
    {"A/B/C", "straight", {L::GC}, false, false},
    {"C", "straight", {L::GC, L::GLA}, false, false},
    // D-2 straight_or_protected_left; route turns left.
    {"D-2", "left", {L::RC}, true, true},
    {"D-2", "left", {L::GC}, true, true},
    {"D-2", "left", {L::RC, L::GLA}, false, false},
    {"D-2", "left", {L::RC, L::YC}, false, true},
    // Not deliverable after profile correction (YC->RC, GC+GLA->GC at D-2), kept as guards.
    {"D-2", "left", {L::YC}, false, true},
    {"D-2", "left", {L::GC, L::GLA}, false, false},
  };
  for (const bool yellow_light_is_go : {true, false}) {
    param_.yellow_light_is_go = yellow_light_is_go;
    for (const auto & c : cases) {
      makeModule(c.turn_direction);
      publish(c.lamps);
      EXPECT_EQ(isStop(), yellow_light_is_go ? c.stop_with_yellow_go : c.stop_without_yellow_go)
        << c.section << " turn=" << c.turn_direction << " lamps=" << c.lamps.size()
        << " yellow_light_is_go=" << yellow_light_is_go;
    }
  }
}

TEST_F(ExperimentalTrafficLightTest, FailOpenPaths)
{
  for (const bool fail_open : {true, false}) {
    param_.competition_fail_open = fail_open;
    const bool stop = !fail_open;

    // 1. Never received for this regulatory element.
    makeModule("straight");
    planner_data_->traffic_light_id_map_raw_.clear();
    EXPECT_EQ(isStop(), stop) << "missing, fail_open=" << fail_open;

    // 2. Last message containing this id is older than tl_state_timeout (1.0 s).
    makeModule("straight");
    publish({element(Element::RED, Element::CIRCLE)}, kRegId, 0.9);
    EXPECT_TRUE(isStop()) << "0.9 s old red is still red";
    publish({element(Element::RED, Element::CIRCLE)}, kRegId, 1.1);
    EXPECT_EQ(isStop(), stop) << "timeout, fail_open=" << fail_open;

    // 3a. Received once, then messages carry another group id (route matcher
    // moved on or fell back to 0). The previous stamp survives, the lamps do not.
    makeModule("straight");
    publish({element(Element::RED, Element::CIRCLE)});
    EXPECT_TRUE(isStop());
    publish({element(Element::RED, Element::CIRCLE)}, 0);
    EXPECT_EQ(isStop(), stop) << "group id switched, fail_open=" << fail_open;

    // 3b. All lamps UNKNOWN, and 3c. no lamps.
    makeModule("straight");
    publish({element(Element::UNKNOWN, Element::UNKNOWN)});
    EXPECT_EQ(isStop(), stop) << "unknown, fail_open=" << fail_open;
    publish(std::vector<Element>{});
    EXPECT_EQ(isStop(), stop) << "empty, fail_open=" << fail_open;
  }
}

TEST_F(ExperimentalTrafficLightTest, DilemmaPassIgnoresLampColor)
{
  // isPassthrough() never looks at the lamp: inside the judge line and within
  // v * yellow_lamp_period it passes, whether the stop came from yellow or red.
  makeModule("straight");
  publish({Lamp::RC});
  ASSERT_TRUE(isStop());
  const double v = 50.0 / 3.6;
  const double judge = judgeLine(v);
  EXPECT_NEAR(judge, 27.1, 0.1);
  EXPECT_TRUE(passthrough(judge - 1.0, v));
  EXPECT_FALSE(passthrough(judge + 1.0, v));
  EXPECT_FALSE(passthrough(v * param_.yellow_lamp_period + 1.0, v));
}

TEST_F(ExperimentalTrafficLightTest, YellowGoDefersFirstStopDecisionToRed)
{
  // Yellow seen 50 m before the line at 50 km/h, yellow lasts 3 s.
  const double v = 50.0 / 3.6;
  const double distance_at_yellow = 50.0;
  const double distance_at_red = distance_at_yellow - 3.0 * v;
  ASSERT_LT(distance_at_red, judgeLine(v));
  ASSERT_GT(distance_at_red, 0.0);

  // Current: no stop at yellow, red arrives inside the judge line -> passes on red.
  param_.yellow_light_is_go = true;
  makeModule("straight");
  publish({Lamp::YC});
  EXPECT_FALSE(isStop());
  publish({Lamp::RC});
  EXPECT_TRUE(isStop());
  EXPECT_TRUE(passthrough(distance_at_red, v));

  // yellow_light_is_go: false -> stop decided at yellow, outside the judge line.
  param_.yellow_light_is_go = false;
  makeModule("straight");
  publish({Lamp::YC});
  EXPECT_TRUE(isStop());
  EXPECT_FALSE(passthrough(distance_at_yellow, v));
}

}  // namespace autoware::behavior_velocity_planner::experimental

int main(int argc, char ** argv) { ros::init(argc, argv, "full_traffic_light_original"); ros::NodeHandle owner; testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
