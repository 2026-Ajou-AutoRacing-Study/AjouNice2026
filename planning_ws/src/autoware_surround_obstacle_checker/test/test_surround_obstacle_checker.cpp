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

#include "../src/node.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <cstdlib>
#include <stdexcept>
#include <thread>
#include <utility>

namespace autoware::surround_obstacle_checker
{
void setTestVehicleInfo(ros::NodeHandle & node)
{
  for (const auto & entry : {
         std::pair{"wheel_radius", 0.39}, {"wheel_width", 0.42}, {"wheel_base", 2.74},
         {"wheel_tread", 1.63}, {"front_overhang", 1.0}, {"rear_overhang", 1.03},
         {"left_overhang", 0.1}, {"right_overhang", 0.1}, {"vehicle_height", 2.5},
         {"max_steer_angle", 0.70}}) {
    node.setParam(entry.first, entry.second);
  }
}

auto generateTestTargetNode() -> std::shared_ptr<SurroundObstacleCheckerNode>
{
  if (std::system(
        "rosparam load \"$(rospack find autoware_surround_obstacle_checker)/config/ros1/"
        "surround_obstacle_checker.yaml\" /surround_original_state") != 0) {
    throw std::runtime_error("failed to load surround obstacle checker ROS1 test parameters");
  }
  ros::NodeHandle node("/surround_original_state");
  setTestVehicleInfo(node);
  return std::make_shared<SurroundObstacleCheckerNode>(node);

}

class SurroundObstacleCheckerNodeTest : public ::testing::Test
{
public:
  SurroundObstacleCheckerNodeTest() : node_{generateTestTargetNode()} {}

  auto isStopRequired(
    const bool is_obstacle_found, const bool is_vehicle_stopped, const State & state,
    const std::optional<ros::Time> & last_obstacle_found_time, const double time_threshold) const
    -> std::pair<bool, std::optional<ros::Time>>
  {
    return node_->isStopRequired(
      is_obstacle_found, is_vehicle_stopped, state, last_obstacle_found_time, time_threshold);
  }

private:
  std::shared_ptr<SurroundObstacleCheckerNode> node_;
};

TEST_F(SurroundObstacleCheckerNodeTest, isStopRequired)
{
  const auto LAST_STOP_TIME = ros::Time::now();

  using namespace std::literals::chrono_literals;
  std::this_thread::sleep_for(500ms);

  {
    constexpr double THRESHOLD = 1.0;
    const auto [is_stop, stop_time] =
      isStopRequired(false, false, State::STOP, LAST_STOP_TIME, THRESHOLD);
    EXPECT_FALSE(is_stop);
    EXPECT_EQ(stop_time, std::nullopt);
  }

  {
    constexpr double THRESHOLD = 1.0;
    const auto [is_stop, stop_time] =
      isStopRequired(false, true, State::PASS, LAST_STOP_TIME, THRESHOLD);
    EXPECT_FALSE(is_stop);
    EXPECT_EQ(stop_time, std::nullopt);
  }

  {
    constexpr double THRESHOLD = 1.0;
    const auto [is_stop, stop_time] =
      isStopRequired(true, true, State::STOP, LAST_STOP_TIME, THRESHOLD);

    ASSERT_TRUE(stop_time.has_value());

    const auto time_diff = ros::Time::now() - stop_time.value();

    EXPECT_TRUE(is_stop);
    EXPECT_NEAR(time_diff.toSec(), 0.0, 1e-3);
  }

  {
    constexpr double THRESHOLD = 1.0;
    const auto [is_stop, stop_time] =
      isStopRequired(false, true, State::STOP, LAST_STOP_TIME, THRESHOLD);

    ASSERT_TRUE(stop_time.has_value());

    const auto time_diff = ros::Time::now() - stop_time.value();

    EXPECT_TRUE(is_stop);
    EXPECT_NEAR(time_diff.toSec(), 0.5, 1e-3);
  }

  {
    constexpr double THRESHOLD = 0.25;
    const auto [is_stop, stop_time] =
      isStopRequired(false, true, State::STOP, LAST_STOP_TIME, THRESHOLD);
    EXPECT_FALSE(is_stop);
    EXPECT_EQ(stop_time, std::nullopt);
  }

  {
    constexpr double THRESHOLD = 1.0;
    const auto [is_stop, stop_time] =
      isStopRequired(false, true, State::STOP, std::nullopt, THRESHOLD);
    EXPECT_FALSE(is_stop);
    EXPECT_EQ(stop_time, std::nullopt);
  }


}
}  // namespace autoware::surround_obstacle_checker

int main(int argc,char ** argv) {
  ros::init(argc,argv,"surround_original_test");
  ::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
