#include <autoware/behavior_velocity_rtc_interface/experimental/scene_module_interface_with_rtc.hpp>
#include <autoware/behavior_velocity_rtc_interface/scene_module_interface_with_rtc.hpp>

#include <gtest/gtest.h>

namespace autoware::behavior_velocity_planner::experimental
{
class TestRTCManager : public SceneModuleManagerInterfaceWithRTC
{
public:
  explicit TestRTCManager(ros::NodeHandle & node)
  : SceneModuleManagerInterfaceWithRTC(node, "rtc_gate", true) {}
  const char * getModuleName() override { return "rtc_gate"; }
  bool registerWithOverride(int64_t id, bool automatic)
  {
    generateUUID(id);
    const auto uuid = getUUID(id);
    updateRTCStatus(uuid, true, State::WAITING_FOR_EXECUTION, 12.0, ros::Time::now(), automatic);
    return rtc_interface_.isRTCEnabled(uuid);
  }
  RequiredSubscriptionInfo getRequiredSubscriptions() const override { return {}; }
private:
  void launchNewModules(const Trajectory &, const ros::Time &, const PlannerData &) override {}
  std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
  getModuleExpiredFunction(const Trajectory &, const PlannerData &) override
  { return [](const std::shared_ptr<SceneModuleInterfaceWithRTC> &) { return false; }; }
};

TEST(RTCManager, RegistersRos1CooperateEndpoints)
{
  ros::NodeHandle node;
  TestRTCManager manager(node);
  for (int i = 0; i < 100; ++i) { ros::spinOnce(); ros::WallDuration(0.01).sleep(); }
  EXPECT_TRUE(ros::service::exists("/planning/cooperate_commands/rtc_gate", false));
  EXPECT_TRUE(ros::service::exists("/planning/enable_auto_mode/rtc_gate", false));
}
}  // namespace autoware::behavior_velocity_planner::experimental

namespace autoware::behavior_velocity_planner
{
class TestStandardRTCModule : public SceneModuleInterfaceWithRTC
{
public:
  explicit TestStandardRTCModule(ros::NodeHandle & node)
  : SceneModuleInterfaceWithRTC(42, node, nullptr, nullptr) {}

  bool modifyPathVelocity(autoware_internal_planning_msgs::PathWithLaneId *) override
  {
    ++plan_calls_;
    return true;
  }
  visualization_msgs::MarkerArray createDebugMarkerArray() override { return {}; }
  motion_utils::VirtualWalls createVirtualWalls() override { return {}; }
  int plan_calls() const { return plan_calls_; }

private:
  int plan_calls_{0};
};

class TestStandardRTCManager : public SceneModuleManagerInterfaceWithRTC
{
public:
  explicit TestStandardRTCManager(ros::NodeHandle & node)
  : SceneModuleManagerInterfaceWithRTC(node, "rtc_gate_standard", true) {}
  const char * getModuleName() override { return "rtc_gate_standard"; }
  bool registerWithOverride(int64_t id, bool automatic)
  {
    generateUUID(id);
    const auto uuid = getUUID(id);
    updateRTCStatus(uuid, true, State::WAITING_FOR_EXECUTION, 12.0, ros::Time::now(), automatic);
    return rtc_interface_.isRTCEnabled(uuid);
  }
  RequiredSubscriptionInfo getRequiredSubscriptions() const override { return {}; }
  void expireNextCycle() { expire_ = true; }
  std::size_t moduleCount() const { return scene_modules_.size(); }
  bool hasModuleUUID() const { return map_uuid_.count(42) != 0; }
  std::shared_ptr<TestStandardRTCModule> module() const { return module_; }
private:
  void launchNewModules(const autoware_internal_planning_msgs::PathWithLaneId &) override
  {
    if (!isModuleRegistered(42)) {
      module_ = std::make_shared<TestStandardRTCModule>(*node_);
      registerModule(module_);
    }
  }
  std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
  getModuleExpiredFunction(const autoware_internal_planning_msgs::PathWithLaneId &) override
  { return [this](const std::shared_ptr<SceneModuleInterfaceWithRTC> &) { return expire_; }; }
  bool expire_{false};
  std::shared_ptr<TestStandardRTCModule> module_;
};

TEST(RTCManager, RegistersStandardRos1CooperateEndpoints)
{
  ros::NodeHandle node;
  TestStandardRTCManager manager(node);
  for (int i = 0; i < 100; ++i) { ros::spinOnce(); ros::WallDuration(0.01).sleep(); }
  EXPECT_TRUE(ros::service::exists("/planning/cooperate_commands/rtc_gate_standard", false));
  EXPECT_TRUE(ros::service::exists("/planning/enable_auto_mode/rtc_gate_standard", false));
}

TEST(RTCManager, BothVariantsPreserveExplicitInitialAutoModeOverride)
{
  ros::NodeHandle node;
  TestStandardRTCManager standard(node);
  experimental::TestRTCManager experimental(node);
  EXPECT_TRUE(standard.registerWithOverride(501, false));
  EXPECT_FALSE(standard.registerWithOverride(502, true));
  EXPECT_TRUE(experimental.registerWithOverride(601, false));
  EXPECT_FALSE(experimental.registerWithOverride(602, true));
}

TEST(RTCManager, StandardManagerPreservesPlanUuidAndExpirationLifecycle)
{
  ros::NodeHandle node;
  TestStandardRTCManager manager(node);
  autoware_internal_planning_msgs::PathWithLaneId path;
  path.header.stamp = ros::Time(52, 0);
  manager.updateSceneModuleInstances(std::shared_ptr<const PlannerData>{}, path);
  ASSERT_EQ(manager.moduleCount(), 1U);
  ASSERT_TRUE(manager.module());
  manager.plan(&path);
  EXPECT_EQ(manager.module()->plan_calls(), 1);
  EXPECT_TRUE(manager.hasModuleUUID());
  manager.expireNextCycle();
  manager.updateSceneModuleInstances(std::shared_ptr<const PlannerData>{}, path);
  EXPECT_EQ(manager.moduleCount(), 0U);
  EXPECT_FALSE(manager.hasModuleUUID());
}

TEST(RTCManager, StandardRtcEnableParameterHonorsGlobalAutoMode)
{
  ros::NodeHandle node;
  const std::string parameter_name{"no_stopping_area/enable_rtc"};
  node.setParam("enable_all_modules_auto_mode", false);
  node.setParam(parameter_name, false);
  EXPECT_FALSE(SceneModuleManagerInterfaceWithRTC::getEnableRTC(node, parameter_name));

  node.setParam(parameter_name, true);
  EXPECT_TRUE(SceneModuleManagerInterfaceWithRTC::getEnableRTC(node, parameter_name));

  node.setParam("enable_all_modules_auto_mode", true);
  EXPECT_FALSE(SceneModuleManagerInterfaceWithRTC::getEnableRTC(node, parameter_name));
}

class HookStandardManager : public TestStandardRTCManager
{
public:
  explicit HookStandardManager(ros::NodeHandle & node) : TestStandardRTCManager(node) {}
  std::vector<int> calls;
protected:
  void setActivation() override { calls.push_back(1); }
  void modifyPathVelocity(autoware_internal_planning_msgs::PathWithLaneId *) override { calls.push_back(2); }
  void sendRTC(const ros::Time &) override { calls.push_back(3); }
};
class HookExperimentalManager : public experimental::TestRTCManager
{
public:
  explicit HookExperimentalManager(ros::NodeHandle & node) : experimental::TestRTCManager(node) {}
  std::vector<int> calls;
protected:
  void setActivation() override { calls.push_back(1); }
  void modifyPathVelocity(experimental::Trajectory &, const std_msgs::Header &,
    const std::vector<geometry_msgs::Point> &, const std::vector<geometry_msgs::Point> &,
    const PlannerData &) override { calls.push_back(2); }
  void sendRTC(const ros::Time &) override { calls.push_back(3); }
};
TEST(RTCManager, BothVariantsInvokeOriginalOverrideHooksInOrder)
{
  ros::NodeHandle node;
  HookStandardManager standard(node);
  autoware_internal_planning_msgs::PathWithLaneId path;
  standard.plan(&path);
  EXPECT_EQ(standard.calls, (std::vector<int>{1,2,3}));
  HookExperimentalManager experimental_manager(node);
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> points(5);
  for (std::size_t i=0; i<points.size(); ++i) {
    points[i].point.pose.position.x = i;
    points[i].point.pose.orientation.w = 1.0;
  }
  auto trajectory = experimental::Trajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory);
  experimental_manager.plan(*trajectory, {}, {}, {}, PlannerData{});
  EXPECT_EQ(experimental_manager.calls, (std::vector<int>{1,2,3}));
}
}  // namespace autoware::behavior_velocity_planner

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "rtc_manager_test", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
