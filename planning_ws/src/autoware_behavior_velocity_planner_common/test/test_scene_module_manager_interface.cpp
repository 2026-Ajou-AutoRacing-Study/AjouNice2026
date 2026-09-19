#include <autoware/behavior_velocity_planner_common/scene_module_manager_interface.hpp>

#include <gtest/gtest.h>

namespace autoware::behavior_velocity_planner
{
namespace
{
class TestModule : public SceneModuleInterface
{
public:
  TestModule(const ros::NodeHandle & node, std::shared_ptr<autoware_utils_debug::TimeKeeper> keeper,
    std::shared_ptr<planning_factor_interface::PlanningFactorInterface> factors)
  : SceneModuleInterface(9, node, std::move(keeper), std::move(factors)) {}
  bool modifyPathVelocity(autoware_internal_planning_msgs::PathWithLaneId * path) override
  {
    path->header.frame_id = "scene_manager";
    planning_factor_interface_->add(
      3.0, geometry_msgs::Pose{}, 42, autoware_internal_planning_msgs::SafetyFactorArray{}, true,
      1.5, 0.0, "manager-oracle");
    return true;
  }
  visualization_msgs::MarkerArray createDebugMarkerArray() override { return {}; }
  motion_utils::VirtualWalls createVirtualWalls() override { return {}; }
};
class TestManager : public SceneModuleManagerInterface<>
{
public:
  explicit TestManager(ros::NodeHandle & node) : SceneModuleManagerInterface(node, "test_scene") {}
  const char * getModuleName() override { return "test_scene"; }
  RequiredSubscriptionInfo getRequiredSubscriptions() const override { return {}; }
protected:
  void launchNewModules(const autoware_internal_planning_msgs::PathWithLaneId &) override
  {
    if (!isModuleRegistered(9)) registerModule(std::make_shared<TestModule>(*node_, time_keeper_, planning_factor_interface_));
  }
  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const autoware_internal_planning_msgs::PathWithLaneId &) override
  { return [](const std::shared_ptr<SceneModuleInterface> &) { return false; }; }
};
TEST(SceneModuleManagerInterface, DispatchesModuleAndPublishesProviderOutputs)
{
  ros::NodeHandle node;
  node.setParam("is_publish_debug_path", true);
  autoware_internal_planning_msgs::PlanningFactorArray::ConstPtr received_factors;
  autoware_internal_planning_msgs::PathWithLaneId::ConstPtr received_debug_path;
  const auto factor_sub = node.subscribe<autoware_internal_planning_msgs::PlanningFactorArray>(
    "/planning/planning_factors/test_scene", 1,
    [&received_factors](const autoware_internal_planning_msgs::PlanningFactorArray::ConstPtr & msg) {
      received_factors = msg;
    });
  const auto debug_path_sub = node.subscribe<autoware_internal_planning_msgs::PathWithLaneId>(
    "/debug/path_with_lane_id/test_scene", 1,
    [&received_debug_path](const autoware_internal_planning_msgs::PathWithLaneId::ConstPtr & msg) {
      received_debug_path = msg;
    });
  TestManager manager(node);
  autoware_internal_planning_msgs::PathWithLaneId path;
  manager.updateSceneModuleInstances(std::make_shared<PlannerData>(), path);
  manager.plan(&path);
  for (int i = 0; i < 20 && (!received_factors || !received_debug_path); ++i) {
    ros::spinOnce();
    ros::WallDuration(0.02).sleep();
  }
  EXPECT_EQ(path.header.frame_id, "scene_manager");
  ASSERT_TRUE(received_factors);
  ASSERT_EQ(received_factors->factors.size(), 1U);
  EXPECT_EQ(received_factors->factors.front().module, "test_scene");
  EXPECT_EQ(received_factors->factors.front().detail, "manager-oracle");
  ASSERT_TRUE(received_debug_path);
  EXPECT_EQ(received_debug_path->header.frame_id, "scene_manager");
}
}  // namespace
}  // namespace autoware::behavior_velocity_planner
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_scene_module_manager_interface");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
