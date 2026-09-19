#include <autoware/motion_velocity_planner/motion_velocity_planner_manager_core.hpp>

#include <gtest/gtest.h>

namespace
{
using autoware::motion_velocity_planner::PluginModuleInterface;
using autoware::motion_velocity_planner::PlannerData;
using autoware::motion_velocity_planner::RequiredSubscriptionInfo;
using autoware::motion_velocity_planner::VelocityPlanningResult;

class TestModule final : public PluginModuleInterface
{
public:
  explicit TestModule(RequiredSubscriptionInfo required, double stop_x)
  : required_(required), stop_x_(stop_x) {}

  void init(ros::NodeHandle &, const std::string & module_name) override { name_ = module_name; ++init_count_; }
  RequiredSubscriptionInfo getRequiredSubscriptions() const override { return required_; }
  void updateParameters(const autoware::motion_velocity_planner::ParameterMap &) override {}
  VelocityPlanningResult plan(
    const std::vector<autoware_planning_msgs::TrajectoryPoint> &,
    const std::vector<autoware_planning_msgs::TrajectoryPoint> &,
    const std::shared_ptr<const PlannerData> &) override
  {
    ++plan_count_;
    VelocityPlanningResult result;
    geometry_msgs::Point stop;
    stop.x = stop_x_;
    result.stop_points.push_back(stop);
    return result;
  }
  std::string getModuleName() const override { return name_; }
  int initCount() const { return init_count_; }
  int planCount() const { return plan_count_; }

private:
  RequiredSubscriptionInfo required_;
  double stop_x_{};
  std::string name_;
  int init_count_{};
  int plan_count_{};
};
}  // namespace

TEST(MotionVelocityPlannerManagerCore, RegistersOnceMergesRequirementsAndPreservesModuleOrder)
{
  ros::NodeHandle node;
  autoware::motion_velocity_planner::MotionVelocityPlannerManagerCore manager;
  const auto first = std::make_shared<TestModule>(
    RequiredSubscriptionInfo{false, true, false, false, false}, 4.0);
  const auto second = std::make_shared<TestModule>(
    RequiredSubscriptionInfo{true, false, false, false, true}, 9.0);
  ASSERT_TRUE(manager.registerModule(node, "first", first));
  ASSERT_TRUE(manager.registerModule(node, "second", second));
  EXPECT_FALSE(manager.registerModule(node, "first", first));
  EXPECT_EQ(manager.size(), 2U);
  EXPECT_EQ(first->initCount(), 1);
  EXPECT_EQ(second->initCount(), 1);
  const auto & required = manager.requiredSubscriptions();
  EXPECT_TRUE(required.traffic_signals);
  EXPECT_TRUE(required.predicted_objects);
  EXPECT_FALSE(required.occupancy_grid_map);
  EXPECT_TRUE(required.no_ground_pointcloud);
  const auto results = manager.planVelocities({}, {}, {});
  ASSERT_EQ(results.size(), 2U);
  EXPECT_DOUBLE_EQ(results.at(0).stop_points.at(0).x, 4.0);
  EXPECT_DOUBLE_EQ(results.at(1).stop_points.at(0).x, 9.0);
  EXPECT_EQ(first->planCount(), 1);
  EXPECT_EQ(second->planCount(), 1);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "motion_velocity_planner_manager_core_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
