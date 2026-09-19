#include <autoware/behavior_path_planner_common/interface/scene_module_visitor.hpp>
#include <gtest/gtest.h>
TEST(SceneModuleVisitor, StartsWithoutModuleSpecificDebugPayload) {
  autoware::behavior_path_planner::SceneModuleVisitor visitor;
  EXPECT_FALSE(visitor.getAvoidanceModuleDebugMsg());
  EXPECT_FALSE(visitor.getStartPlannerModuleDebugMsg());
}
