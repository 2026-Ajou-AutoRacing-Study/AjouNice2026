#include <autoware/behavior_path_goal_planner_module/fixed_goal_planner_base.hpp>

#include <gtest/gtest.h>

namespace
{
class EchoFixedGoalPlanner : public autoware::behavior_path_planner::FixedGoalPlannerBase
{
public:
  autoware::behavior_path_planner::BehaviorModuleOutput plan(
    const std::shared_ptr<const autoware::behavior_path_planner::PlannerData> &) const override
  {
    return getPreviousModuleOutput();
  }
};
}  // namespace

TEST(FixedGoalPlannerBase, RetainsUpstreamOutputAcrossConcretePlannerBoundary)
{
  EchoFixedGoalPlanner planner;
  autoware::behavior_path_planner::BehaviorModuleOutput upstream{};
  upstream.path.header.frame_id = "map";
  planner.setPreviousModuleOutput(upstream);

  const auto output = planner.plan(nullptr);
  EXPECT_EQ(output.path.header.frame_id, "map");
  EXPECT_EQ(planner.getPreviousModuleOutput().path.header.frame_id, "map");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
