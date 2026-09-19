#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>

#include <gtest/gtest.h>

namespace
{
using autoware::motion_velocity_planner::ParameterMap;
using autoware::motion_velocity_planner::PlannerData;
using autoware::motion_velocity_planner::PluginModuleInterface;
using autoware::motion_velocity_planner::RequiredSubscriptionInfo;
using autoware::motion_velocity_planner::VelocityPlanningResult;

class TestPlugin final : public PluginModuleInterface
{
public:
  void init(ros::NodeHandle & node, const std::string & module_name) override
  {
    node_namespace = node.getNamespace();
    name = module_name;
  }
  RequiredSubscriptionInfo getRequiredSubscriptions() const override { return required; }
  void updateParameters(const ParameterMap & parameters) override { last_parameter_count = parameters.size(); }
  VelocityPlanningResult plan(
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & raw,
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & smoothed,
    const std::shared_ptr<const PlannerData> &) override
  {
    raw_points = raw.size();
    smoothed_points = smoothed.size();
    return {};
  }
  std::string getModuleName() const override { return name; }

  RequiredSubscriptionInfo required{true, false, true, false};
  std::string name;
  std::string node_namespace;
  size_t last_parameter_count{};
  size_t raw_points{};
  size_t smoothed_points{};
};
}  // namespace

TEST(MotionVelocityPluginModuleInterface, PreservesLifecycleParametersAndImmutablePlanInputs)
{
  ros::NodeHandle node("mvp_abi_test");
  TestPlugin plugin;
  plugin.init(node, "obstacle_stop");

  ParameterMap parameters;
  parameters.emplace("threshold", XmlRpc::XmlRpcValue(2.5));
  plugin.updateParameters(parameters);
  const std::vector<autoware_planning_msgs::TrajectoryPoint> raw(2);
  const std::vector<autoware_planning_msgs::TrajectoryPoint> smoothed(3);
  const std::shared_ptr<const PlannerData> absent_data;
  plugin.plan(raw, smoothed, absent_data);

  EXPECT_EQ(plugin.getModuleName(), "obstacle_stop");
  EXPECT_EQ(plugin.getShortModuleName(), "obstacle_stop");
  EXPECT_EQ(plugin.last_parameter_count, 1U);
  EXPECT_EQ(plugin.raw_points, 2U);
  EXPECT_EQ(plugin.smoothed_points, 3U);
  const auto required = plugin.getRequiredSubscriptions();
  EXPECT_TRUE(required.traffic_signals);
  EXPECT_TRUE(required.occupancy_grid_map);
  EXPECT_FALSE(required.predicted_objects);
  EXPECT_FALSE(required.no_ground_pointcloud);
}

TEST(MotionVelocityPluginModuleInterface, MergesOptionalSubscriptionRequirements)
{
  RequiredSubscriptionInfo aggregate{true, false, false, false};
  aggregate.update(RequiredSubscriptionInfo{false, true, true, false});
  aggregate.update(RequiredSubscriptionInfo{false, false, false, false, true});

  EXPECT_TRUE(aggregate.traffic_signals);
  EXPECT_TRUE(aggregate.predicted_objects);
  EXPECT_TRUE(aggregate.occupancy_grid_map);
  EXPECT_TRUE(aggregate.no_ground_pointcloud);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "motion_velocity_plugin_module_interface_test", ros::init_options::AnonymousName);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
