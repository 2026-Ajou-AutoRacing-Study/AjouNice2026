#include <autoware/behavior_path_planner/behavior_path_planner_core.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cstdlib>
#include <string>

namespace
{
TEST(BehaviorPathPlannerCore, EnablesAvoidanceByLaneChangeOnlyWithRenderedParameters)
{
  constexpr const char * kNamespace = "/bpp_avoidance_by_lane_change_slot";
  const std::string load_command =
    std::string("rosparam load ") + STATIC_OBSTACLE_AVOIDANCE_CONFIG + " " + kNamespace +
    " && rosparam load " + AVOIDANCE_BY_LANE_CHANGE_CONFIG + " " + kNamespace +
    " && python3 " + LANE_CHANGE_PARAMETER_LOADER + " " + kNamespace;
  ASSERT_EQ(std::system(load_command.c_str()), 0) << "candidate parameter closure failed";

  ros::NodeHandle private_node(kNamespace);
  private_node.setParam("enable_avoidance_by_lane_change_module", true);
  private_node.setParam("side_shift/min_distance_to_start_shifting", 1.0);
  private_node.setParam("side_shift/min_shifting_distance", 2.0);
  private_node.setParam("side_shift/min_shifting_speed", 1.0);
  autoware::behavior_path_planner::BehaviorPathPlannerCore core(&private_node);
  EXPECT_TRUE(core.isInitialized());
  EXPECT_TRUE(core.avoidanceByLaneChangeEnabled());
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "bpp_avoidance_by_lane_change_slot_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
