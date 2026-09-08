#include <autoware/behavior_velocity_planner_common/plugin_wrapper.hpp>

#include <gtest/gtest.h>

namespace autoware::behavior_velocity_planner
{
namespace
{
class FakeSceneManager
{
public:
  explicit FakeSceneManager(ros::NodeHandle &) {}

  RequiredSubscriptionInfo getRequiredSubscriptions()
  {
    RequiredSubscriptionInfo info;
    info.traffic_signals = true;
    info.no_ground_pointcloud = true;
    return info;
  }

  void plan(autoware_internal_planning_msgs::PathWithLaneId * path)
  {
    path->header.frame_id = "plugin_wrapper";
  }

  void updateSceneModuleInstances(
    const std::shared_ptr<const PlannerData> &, const autoware_internal_planning_msgs::PathWithLaneId &)
  {
  }

  const char * getModuleName() { return "fake_scene"; }
};

TEST(PluginWrapper, PreservesLifecycleSubscriptionAndMutablePathContract)
{
  if (!ros::isInitialized()) {
    int argc = 0;
    ros::init(argc, nullptr, "plugin_wrapper_test", ros::init_options::AnonymousName);
  }
  ros::NodeHandle node;
  PluginWrapper<FakeSceneManager> wrapper;
  wrapper.init(node);

  const auto subscriptions = wrapper.getRequiredSubscriptions();
  EXPECT_TRUE(subscriptions.traffic_signals);
  EXPECT_TRUE(subscriptions.no_ground_pointcloud);
  EXPECT_FALSE(subscriptions.predicted_objects);

  autoware_internal_planning_msgs::PathWithLaneId path;
  wrapper.updateSceneModuleInstances(std::make_shared<PlannerData>(), path);
  wrapper.plan(&path);
  EXPECT_EQ(path.header.frame_id, "plugin_wrapper");
  EXPECT_STREQ(wrapper.getModuleName(), "fake_scene");
}
}  // namespace
}  // namespace autoware::behavior_velocity_planner
