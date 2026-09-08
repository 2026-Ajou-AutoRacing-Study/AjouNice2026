#include <autoware/behavior_path_goal_planner_module/manager.hpp>
#include <autoware/behavior_path_goal_planner_module/goal_planner_module.hpp>
#include <gtest/gtest.h>
#include <chrono>
#include <thread>

namespace bpp = autoware::behavior_path_planner;

TEST(FullGoalManager, OriginalParametersAndModuleConstruction)
{
  ros::NodeHandle node("/full_goal_test");
  const auto parameters = bpp::GoalPlannerModuleManager::initGoalPlannerParameters(
    &node, "goal_planner.");
  double expected = 0.0;
  ASSERT_TRUE(node.getParam("goal_planner/goal_search/forward_goal_search_length", expected));
  EXPECT_EQ(parameters.forward_goal_search_length, expected);
  bpp::GoalPlannerModuleManager manager;
  ASSERT_TRUE(manager.initialize(&node));
  EXPECT_FALSE(manager.isSimultaneousExecutableAsApprovedModule());
  auto module = manager.createNewSceneModuleInstance();
  ASSERT_TRUE(std::dynamic_pointer_cast<bpp::GoalPlannerModule>(
    std::shared_ptr<bpp::SceneModuleCore>(std::move(module))));
  // Another instance stays alive through both timer periods without requests.
  auto running = manager.createNewSceneModuleInstance();
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));
  running.reset();
}

TEST(FullGoalManager, RequiredParameterMissingOrWrongTypeIsRejected)
{
  ros::NodeHandle node("/full_goal_test");
  const std::string key = "goal_planner/th_stopped_velocity";
  double original;
  ASSERT_TRUE(node.getParam(key, original));
  node.deleteParam(key);
  EXPECT_THROW(bpp::GoalPlannerModuleManager::initGoalPlannerParameters(
    &node, "goal_planner."), std::runtime_error);
  node.setParam(key, std::string("0.1"));
  EXPECT_THROW(bpp::GoalPlannerModuleManager::initGoalPlannerParameters(
    &node, "goal_planner."), std::runtime_error);
  node.setParam(key, original);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "full_goal_manager_test", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
