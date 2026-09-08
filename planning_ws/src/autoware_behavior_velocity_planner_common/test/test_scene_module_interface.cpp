#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>

#include <gtest/gtest.h>

namespace autoware::behavior_velocity_planner
{
namespace
{
class TestSceneModule : public SceneModuleInterface
{
public:
  explicit TestSceneModule(const ros::NodeHandle & node)
  : SceneModuleInterface(42, node, nullptr, nullptr)
  {
  }
  bool modifyPathVelocity(autoware_internal_planning_msgs::PathWithLaneId *) override { return true; }
  visualization_msgs::MarkerArray createDebugMarkerArray() override { return {}; }
  motion_utils::VirtualWalls createVirtualWalls() override { return {}; }
  void addObject(const geometry_msgs::Pose & pose, const autoware_perception_msgs::Shape & shape)
  {
    setObjectsOfInterestData(pose, shape, objects_of_interest_marker_interface::ColorName::GREEN);
  }
  size_t egoSegment(const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points) const
  {
    return findEgoSegmentIndex(points);
  }
  std::vector<int64_t> getLaneletIds() const override { return {7}; }
};

TEST(SceneModuleInterface, PreservesModuleStateObjectsAndNearestSegmentContract)
{
  ros::NodeHandle node;
  TestSceneModule module(node);
  auto data = std::make_shared<PlannerData>();
  auto odometry = std::make_shared<geometry_msgs::PoseStamped>();
  odometry->pose.position.x = 0.2;
  odometry->pose.orientation.w = 1.0;
  data->current_odometry = odometry;
  data->ego_nearest_dist_threshold = 3.0;
  data->ego_nearest_yaw_threshold = 1.57;
  module.setPlannerData(data);

  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> points(3);
  points.at(0).point.pose.position.x = 0.0;
  points.at(0).point.pose.orientation.w = 1.0;
  points.at(1).point.pose.position.x = 1.0;
  points.at(1).point.pose.orientation.w = 1.0;
  points.at(2).point.pose.position.x = 2.0;
  points.at(2).point.pose.orientation.w = 1.0;
  EXPECT_EQ(module.getModuleId(), 42);
  EXPECT_EQ(module.egoSegment(points), 0U);

  geometry_msgs::Pose pose;
  pose.position.x = 4.0;
  autoware_perception_msgs::Shape shape;
  module.addObject(pose, shape);
  ASSERT_EQ(module.getObjectsOfInterestData().size(), 1U);
  EXPECT_DOUBLE_EQ(module.getObjectsOfInterestData().front().pose.position.x, 4.0);
  EXPECT_EQ(module.getObjectsOfInterestData().front().color,
    objects_of_interest_marker_interface::ColorName::GREEN);
  module.clearObjectsOfInterestData();
  EXPECT_TRUE(module.getObjectsOfInterestData().empty());
}
}  // namespace
}  // namespace autoware::behavior_velocity_planner

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_scene_module_interface");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
