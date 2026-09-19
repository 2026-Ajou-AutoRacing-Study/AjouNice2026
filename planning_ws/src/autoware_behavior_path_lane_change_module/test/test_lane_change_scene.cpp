// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_path_lane_change_module/manager.hpp"
#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_lane_change_module/structs/data.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"
#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware_test_utils/autoware_test_utils.hpp"
#include "autoware_test_utils/mock_data_parser.hpp"

#include <ros/package.h>
#include <ros/ros.h>

#include <gtest/gtest.h>

#include <limits>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>

using autoware::behavior_path_planner::FilteredLanesObjects;
using autoware::behavior_path_planner::LaneChangeModuleManager;
using autoware::behavior_path_planner::LaneChangeModuleType;
using autoware::behavior_path_planner::NormalLaneChange;
using autoware::behavior_path_planner::PlannerData;
using autoware::behavior_path_planner::lane_change::CommonDataPtr;
using autoware::behavior_path_planner::lane_change::LCParamPtr;
using autoware::behavior_path_planner::lane_change::RouteHandlerPtr;
using autoware::route_handler::Direction;
using autoware::route_handler::RouteHandler;
using autoware::test_utils::get_absolute_path_to_config;
using autoware::test_utils::get_absolute_path_to_lanelet_map;
using autoware::test_utils::get_absolute_path_to_route;
using autoware_internal_planning_msgs::PathWithLaneId;
using autoware_map_msgs::LaneletMapBin;
using autoware_perception_msgs::PredictedObjects;
using autoware_planning_msgs::LaneletRoute;
using geometry_msgs::Pose;

class TestNormalLaneChange : public ::testing::Test
{
public:
  void SetUp() override
  {
    init_param();
    init_module();
  }

  void init_param()
  {
    ros::NodeHandle node("/test_lane_change_scene");
    planner_data_->init_parameters(node);
    lc_param_ptr_ = LaneChangeModuleManager::set_params(&node, name_);
    planner_data_->route_handler = init_route_handler();

    ego_pose_ = autoware::test_utils::createPose(-50.0, 1.75, 0.0, 0.0, 0.0, 0.0);
    planner_data_->self_odometry = set_odometry(ego_pose_);
    const auto objects_file =
      ros::package::getPath("autoware_behavior_path_lane_change_module") +
      "/test_data/test_object_filter.yaml";

    YAML::Node yaml_node = YAML::LoadFile(objects_file);
    const auto objects = autoware::test_utils::parse<PredictedObjects>(yaml_node);
    planner_data_->dynamic_object = boost::make_shared<PredictedObjects>(objects);
  }

  void init_module()
  {
    normal_lane_change_ =
      std::make_shared<NormalLaneChange>(lc_param_ptr_, lc_type_, lc_direction_);
    normal_lane_change_->setData(planner_data_);
    set_previous_approved_path();
  }

  [[nodiscard]] const CommonDataPtr & get_common_data_ptr() const
  {
    return normal_lane_change_->common_data_ptr_;
  }

  [[nodiscard]] RouteHandlerPtr init_route_handler() const
  {
    std::string autoware_route_handler_dir{"autoware_route_handler"};
    std::string lane_change_right_test_route_filename{"lane_change_test_route.yaml"};
    std::string lanelet_map_filename{"2km_test.osm"};
    const auto lanelet2_path =
      get_absolute_path_to_lanelet_map(test_utils_dir_, lanelet_map_filename);
    const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(lanelet2_path, 5.0);
    auto route_handler_ptr = std::make_shared<RouteHandler>(map_bin_msg);
    const auto rh_test_route =
      ros::package::getPath("autoware_behavior_path_lane_change_module") + "/test_data/" + lane_change_right_test_route_filename;
    if (
      const auto route_opt =
        autoware::test_utils::parse<std::optional<LaneletRoute>>(rh_test_route)) {
      auto route = *route_opt;
      // The fixture starts on 4765 and asks the NORMAL/RIGHT module to take
      // its parallel route lane.  In ROS1 the preferred primitive is used
      // directly by RouteHandler, so retain the source route alternatives
      // but make that requested right-side target explicit instead of
      // declaring the already-current lane preferred.
      if (route.segments.empty() || route.segments.front().primitives.size() < 2U) {
        return route_handler_ptr;
      }
      route.segments.front().preferred_primitive = route.segments.front().primitives.at(1);
      route_handler_ptr->setRoute(route);
    }

    return route_handler_ptr;
  }

  static nav_msgs::OdometryPtr set_odometry(const Pose & pose)
  {
    nav_msgs::Odometry odom;
    odom.pose.pose = pose;
    return boost::make_shared<nav_msgs::Odometry>(odom);
  }

  [[nodiscard]] FilteredLanesObjects & get_filtered_objects() const
  {
    return normal_lane_change_->filtered_objects_;
  }

  void set_previous_approved_path()
  {
    previous_approved_path_ = create_previous_approved_path();
    normal_lane_change_->prev_module_output_.path = previous_approved_path_;
  }

  [[nodiscard]] PathWithLaneId create_previous_approved_path() const
  {
    const auto & common_data_ptr = get_common_data_ptr();
    const auto & route_handler_ptr = common_data_ptr->route_handler_ptr;
    lanelet::ConstLanelet closest_lane;
    const auto current_pose = planner_data_->self_odometry->pose.pose;
    route_handler_ptr->getClosestLaneletWithinRoute(current_pose, &closest_lane);
    const auto backward_distance = common_data_ptr->bpp_param_ptr->backward_path_length;
    const auto forward_distance = common_data_ptr->bpp_param_ptr->forward_path_length;
    const auto current_lanes = route_handler_ptr->getLaneletSequence(
      closest_lane, current_pose, backward_distance, forward_distance);

    return route_handler_ptr->getCenterLinePath(
      current_lanes, 0.0, std::numeric_limits<double>::max());
  }

  void TearDown() override
  {
    normal_lane_change_ = nullptr;
    lc_param_ptr_ = nullptr;
    planner_data_ = nullptr;
  }

  LCParamPtr lc_param_ptr_;
  std::shared_ptr<NormalLaneChange> normal_lane_change_;
  std::shared_ptr<PlannerData> planner_data_ = std::make_shared<PlannerData>();
  LaneChangeModuleType lc_type_{LaneChangeModuleType::NORMAL};
  Direction lc_direction_{Direction::RIGHT};
  std::string name_{"test_lane_change_scene"};
  std::string test_utils_dir_{"autoware_test_utils"};
  Pose ego_pose_;
  PathWithLaneId previous_approved_path_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "lane_change_scene_ros1_oracle");
  const int load_result = std::system(
    "python3 \"$(rospack find autoware_behavior_path_lane_change_module)/scripts/"
    "load_ros1_parameters.py\" /test_lane_change_scene "
    "--source \"$(rospack find autoware_behavior_path_lane_change_module)/config/"
    "lane_change.param.yaml\" "
    "--source \"$(rospack find autoware_test_utils)/config/test_common.param.yaml\" "
    "--source \"$(rospack find autoware_test_utils)/config/test_nearest_search.param.yaml\" "
    "--source \"$(rospack find autoware_test_utils)/config/test_vehicle_info.param.yaml\" "
    "--source \"$(rospack find autoware_behavior_path_planner)/config/"
    "behavior_path_planner.param.yaml\" "
    "--source \"$(rospack find autoware_behavior_path_planner)/config/"
    "drivable_area_expansion.param.yaml\" "
    "--source \"$(rospack find autoware_behavior_path_planner)/config/"
    "scene_module_manager.param.yaml\"");
  if (load_result != 0) return EXIT_FAILURE;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST_F(TestNormalLaneChange, testBaseClassInitialize)
{
  const auto type = normal_lane_change_->getModuleType();
  const auto type_str = normal_lane_change_->getModuleTypeStr();

  ASSERT_EQ(type, LaneChangeModuleType::NORMAL);
  const auto is_type_str = type_str == "NORMAL";
  ASSERT_TRUE(is_type_str);

  ASSERT_EQ(normal_lane_change_->getDirection(), Direction::RIGHT);

  ASSERT_TRUE(get_common_data_ptr());

  ASSERT_TRUE(get_common_data_ptr()->is_data_available());
  ASSERT_FALSE(get_common_data_ptr()->is_lanes_available());
}

TEST_F(TestNormalLaneChange, testUpdateLanes)
{
  constexpr auto is_approved = true;

  normal_lane_change_->update_lanes(is_approved);

  ASSERT_FALSE(get_common_data_ptr()->is_lanes_available());

  normal_lane_change_->update_lanes(!is_approved);

  ASSERT_FALSE(previous_approved_path_.points.empty());
  ASSERT_FALSE(previous_approved_path_.points.front().lane_ids.empty());

  // Keep the upstream assertion below, but make the three source early-return
  // boundaries observable in the ROS1 oracle.  This does not synthesize a
  // lane or change the fixture: it proves that the map/route path supplies a
  // current lane and that the same target-lane query used by update_lanes()
  // can traverse the right-side routing relation at the original -50 m pose.
  auto current_lanes = autoware::behavior_path_planner::utils::getCurrentLanesFromPath(
    previous_approved_path_, planner_data_);
  autoware::behavior_path_planner::utils::lane_change::trim_preferred_after_alternative(
    current_lanes, get_common_data_ptr()->route_handler_ptr->getPreferredLanelets());
  ASSERT_FALSE(current_lanes.empty());
  std::ostringstream lane_relation_trace;
  const auto routing_graph = get_common_data_ptr()->route_handler_ptr->getRoutingGraphPtr();
  for (const auto & lane : current_lanes) {
    const auto num_to_preferred = get_common_data_ptr()->route_handler_ptr->getNumLaneToPreferredLane(
      lane, Direction::RIGHT);
    const auto right_lane = routing_graph->right(lane);
    const auto adjacent_right_lane = routing_graph->adjacentRight(lane);
    lane_relation_trace << "lane=" << lane.id() << " preferred_delta=" << num_to_preferred
                        << " direct_right=" << (right_lane ? right_lane->id() : -1)
                        << " adjacent_right="
                        << (adjacent_right_lane ? adjacent_right_lane->id() : -1) << "; ";
  }
  SCOPED_TRACE(lane_relation_trace.str());
  const auto target_lane =
    autoware::behavior_path_planner::utils::lane_change::get_lane_change_target_lane(
      get_common_data_ptr(), current_lanes);
  ASSERT_TRUE(target_lane.has_value());
  const auto target_lanes = get_common_data_ptr()->route_handler_ptr->getLaneletSequence(
    target_lane.value(), ego_pose_, 0.0,
    get_common_data_ptr()->bpp_param_ptr->forward_path_length);
  ASSERT_FALSE(target_lanes.empty());

  ASSERT_TRUE(get_common_data_ptr()->is_lanes_available());
}

TEST_F(TestNormalLaneChange, PendingGeometryClearsWhenCurrentPathDisappears)
{
  normal_lane_change_->update_lanes(false);
  ASSERT_TRUE(get_common_data_ptr()->is_lanes_available());
  normal_lane_change_->setPreviousModuleOutput(autoware::behavior_path_planner::BehaviorModuleOutput{});
  // An approved maneuver retains its original lane geometry.
  normal_lane_change_->update_lanes(true);
  EXPECT_TRUE(get_common_data_ptr()->is_lanes_available());
  normal_lane_change_->update_lanes(false);
  EXPECT_FALSE(get_common_data_ptr()->is_lanes_available());
  EXPECT_TRUE(get_common_data_ptr()->lanes_ptr->current.empty());
  EXPECT_TRUE(get_common_data_ptr()->lanes_ptr->target.empty());
  EXPECT_FALSE(get_common_data_ptr()->transient_data.has_valid_lc_distance);
  EXPECT_EQ(normal_lane_change_->isLaneChangeRequired(),
    std::optional<std::string>("LC unavailable: current lane sequence empty."));
}

TEST_F(TestNormalLaneChange, PendingGeometryClearsWhenTargetDisappears)
{
  normal_lane_change_->update_lanes(false);
  ASSERT_TRUE(get_common_data_ptr()->is_lanes_available());
  // The fixture requires RIGHT; there is no preferred target to its LEFT.
  get_common_data_ptr()->direction = Direction::LEFT;
  normal_lane_change_->update_lanes(false);
  EXPECT_FALSE(get_common_data_ptr()->is_lanes_available());
  EXPECT_TRUE(get_common_data_ptr()->lanes_ptr->target.empty());
  EXPECT_TRUE(get_common_data_ptr()->target_lanes_path.points.empty());
}

TEST_F(TestNormalLaneChange, testGetPathWhenInvalid)
{
  constexpr auto is_approved = true;
  normal_lane_change_->update_lanes(!is_approved);
  normal_lane_change_->update_filtered_objects();
  normal_lane_change_->update_transient_data(!is_approved);
  normal_lane_change_->updateLaneChangeStatus();
  const auto & lc_status = normal_lane_change_->getLaneChangeStatus();

  ASSERT_FALSE(lc_status.is_valid_path);
}

// TODO(Azu, Quda): Fix this test
TEST_F(TestNormalLaneChange, DISABLED_testFilteredObjects)
{
  constexpr auto is_approved = true;
  ego_pose_ = autoware::test_utils::createPose(1.0, 1.75, 0.0, 0.0, 0.0, 0.0);
  planner_data_->self_odometry = set_odometry(ego_pose_);
  set_previous_approved_path();

  normal_lane_change_->update_lanes(!is_approved);
  normal_lane_change_->update_filtered_objects();

  const auto & filtered_objects = get_filtered_objects();

  const auto filtered_size =
    filtered_objects.current_lane.size() + filtered_objects.target_lane_leading.size() +
    filtered_objects.target_lane_trailing.size() + filtered_objects.others.size();
  EXPECT_EQ(filtered_size, planner_data_->dynamic_object->objects.size());
  EXPECT_EQ(filtered_objects.current_lane.size(), 1);
  EXPECT_EQ(filtered_objects.target_lane_leading.size(), 2);
  EXPECT_EQ(filtered_objects.target_lane_trailing.size(), 0);
  EXPECT_EQ(filtered_objects.others.size(), 1);
}

TEST_F(TestNormalLaneChange, testGetPathWhenValid)
{
  constexpr auto is_approved = true;
  ego_pose_ = autoware::test_utils::createPose(1.0, 1.75, 0.0, 0.0, 0.0, 0.0);
  planner_data_->self_odometry = set_odometry(ego_pose_);
  normal_lane_change_->setData(planner_data_);
  set_previous_approved_path();
  normal_lane_change_->update_lanes(!is_approved);
  normal_lane_change_->update_filtered_objects();
  normal_lane_change_->update_transient_data(!is_approved);
  const auto err = normal_lane_change_->isLaneChangeRequired();

  ASSERT_FALSE(err);

  normal_lane_change_->updateLaneChangeStatus();
  const auto & lc_status = normal_lane_change_->getLaneChangeStatus();

  ASSERT_TRUE(lc_status.is_valid_path);
}

TEST_F(TestNormalLaneChange, ExtendActualOutputRatherThanStoredCandidate)
{
  normal_lane_change_->update_lanes(false);
  const auto & data = get_common_data_ptr();
  ASSERT_FALSE(data->lanes_ptr->target.empty());
  const auto output = data->route_handler_ptr->getCenterLinePath(
    data->lanes_ptr->target, 0.0, 15.0);
  ASSERT_GT(output.points.size(), 1U);
  // No selected candidate exists: terminal/waiting output must still extend.
  ASSERT_TRUE(normal_lane_change_->getLaneChangePath().path.points.empty());
  EXPECT_FALSE(normal_lane_change_->extendPath());
  const auto extension = normal_lane_change_->extendPath(output);
  ASSERT_TRUE(extension);
  ASSERT_GT(extension->points.size(), 1U);
  const auto & join = output.points.back().point.pose.position;
  const auto & start = extension->points.front().point.pose.position;
  EXPECT_NEAR(join.x, start.x, 0.1);
  EXPECT_NEAR(join.y, start.y, 0.1);
  EXPECT_FALSE(normal_lane_change_->extendPath(PathWithLaneId{}));
}

TEST_F(TestNormalLaneChange, LeftLaneChangeAlsoContainsFullManeuver)
{
  const auto file = ros::package::getPath("autoware_behavior_path_lane_change_module") +
    "/test_data/lane_change_test_route.yaml";
  const auto parsed = autoware::test_utils::parse<std::optional<LaneletRoute>>(file);
  ASSERT_TRUE(parsed);
  auto route = *parsed;
  for (auto & section : route.segments) {
    section.preferred_primitive = section.primitives.front();
  }
  route.goal_pose.position.y = 1.75;
  planner_data_->route_handler->setRoute(route);
  ego_pose_ = autoware::test_utils::createPose(1.0, -1.75, 0.0, 0.0, 0.0, 0.0);
  planner_data_->self_odometry = set_odometry(ego_pose_);
  lc_direction_ = Direction::LEFT;
  init_module();
  normal_lane_change_->update_lanes(false);
  normal_lane_change_->update_filtered_objects();
  normal_lane_change_->update_transient_data(false);
  const auto required_error = normal_lane_change_->isLaneChangeRequired();
  ASSERT_FALSE(required_error) << required_error.value_or("");
  normal_lane_change_->updateLaneChangeStatus();
  const auto & status = normal_lane_change_->getLaneChangeStatus();
  ASSERT_TRUE(status.is_valid_path);
  EXPECT_TRUE(autoware::behavior_path_planner::utils::lane_change::contains_lane_change_maneuver(
    status.lane_change_path));
}

TEST_F(TestNormalLaneChange, PendingSignalTimeCountsWithoutAValidCandidate)
{
  lc_param_ptr_->trajectory.max_prepare_duration = 4.0;
  lc_param_ptr_->trajectory.min_prepare_duration = 2.0;
  auto odom = set_odometry(ego_pose_);
  odom->twist.twist.linear.x = 10.0;
  planner_data_->self_odometry = odom;
  normal_lane_change_->setData(planner_data_);
  normal_lane_change_->update_lanes(false);
  ASSERT_FALSE(normal_lane_change_->getLaneChangeStatus().is_valid_path);
  ros::Time::setNow(ros::Time(100));
  normal_lane_change_->get_current_turn_signal_info();
  ros::Time::setNow(ros::Time(103));
  normal_lane_change_->update_transient_data(false);
  EXPECT_DOUBLE_EQ(get_common_data_ptr()->transient_data.lane_change_prepare_duration, 2.0);
  ros::Time::init();
}
