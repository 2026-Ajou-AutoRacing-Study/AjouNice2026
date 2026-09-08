#include <autoware/behavior_path_planner_common/data_manager.hpp>
#include <gtest/gtest.h>
TEST(DataManagerSourceSurface, PreservesDataOwnershipDefaults) {
  autoware::behavior_path_planner::BoolStamped value(true);
  EXPECT_TRUE(value.data);
  autoware::behavior_path_planner::ModuleNameStamped module;
  EXPECT_EQ(module.module_name, "NONE");
  autoware::behavior_path_planner::PlannerData data;
  EXPECT_TRUE(data.prev_output_path);
}
