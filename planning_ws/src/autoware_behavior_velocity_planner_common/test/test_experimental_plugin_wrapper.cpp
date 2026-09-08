#include <autoware/behavior_velocity_planner_common/experimental/plugin_wrapper.hpp>
#include <autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp>

#include <gtest/gtest.h>

namespace avp = autoware::behavior_velocity_planner;
namespace experimental_v = autoware::behavior_velocity_planner::experimental;

namespace
{
class TestManager : public experimental_v::SceneModuleManagerInterface<>
{
public:
  explicit TestManager(ros::NodeHandle & node) : SceneModuleManagerInterface(node, "experimental_test") {}
  const char * getModuleName() override { return "experimental_test"; }
  avp::RequiredSubscriptionInfo getRequiredSubscriptions() const override { return {}; }

private:
  void launchNewModules(const experimental_v::Trajectory &, const ros::Time &, const avp::PlannerData &) override {}
  std::function<bool(const std::shared_ptr<experimental_v::SceneModuleInterface> &)> getModuleExpiredFunction(
    const experimental_v::Trajectory &, const avp::PlannerData &) override
  {
    return [](const std::shared_ptr<experimental_v::SceneModuleInterface> &) { return false; };
  }
};

TEST(ExperimentalPluginWrapper, InitializesRos1TrajectoryLifecycle)
{
  ros::NodeHandle node;
  experimental_v::PluginWrapper<TestManager> plugin;
  plugin.init(node);
  EXPECT_STREQ(plugin.getModuleName(), "experimental_test");
  const auto required = plugin.getRequiredSubscriptions();
  EXPECT_FALSE(required.predicted_objects);
  EXPECT_FALSE(required.traffic_signals);
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "experimental_plugin_wrapper_test", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
