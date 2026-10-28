#include <autoware/behavior_path_dynamic_obstacle_avoidance_module/scene.hpp>
#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp>

#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/is_valid.hpp>
#include <gtest/gtest.h>

namespace autoware::behavior_path_planner
{
struct DynamicObstacleAvoidanceTestAccess
{
  static void targets(DynamicObstacleAvoidanceModule & module, const PredictedObject & object)
  {
    module.target_objects_.clear();
    module.target_objects_.emplace_back(object, 0.0, 0.0, false, std::nullopt);
  }
};

namespace
{
geometry_msgs::Pose pose(const double x, const double y)
{
  geometry_msgs::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation.w = 1.0;
  return p;
}

PredictedObject object(const double x = 10.0, const double y = 2.0)
{
  PredictedObject o;
  o.object_id.uuid[0] = 42;
  o.classification.resize(1);
  o.classification.front().label = autoware_perception_msgs::ObjectClassification::UNKNOWN;
  o.kinematics.initial_pose_with_covariance.pose = pose(x, y);
  o.shape.type = Shape::BOUNDING_BOX;
  o.shape.dimensions.x = 2.0;
  o.shape.dimensions.y = 1.0;
  return o;
}

class IndependentConstraints : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!ros::isInitialized()) {
      ros::M_string remaps;
      ros::init(remaps, "test_doa_independent_constraints", ros::init_options::AnonymousName);
    }
    node = std::make_unique<ros::NodeHandle>();
    parameters = std::make_shared<DynamicAvoidanceParameters>();
    parameters->avoid_unknown = true;
    parameters->avoid_car = true;
    parameters->expand_drivable_area = false;
    parameters->margin_distance_around_pedestrian = 0.8;
    parameters->lat_offset_from_obstacle = 0.3;
    parameters->end_time_to_consider = 1.0;
    parameters->polygon_generation_method = PolygonGenerationMethod::OBJECT_PATH_BASE;
    module = std::make_unique<DynamicObstacleAvoidanceModule>(
      "dynamic_obstacle_avoidance", *node, parameters, SceneModuleInterface::RTCInterfaceMap{},
      markers, nullptr);
    for (int x = 0; x <= 30; ++x) {
      PathPointWithLaneId p;
      p.point.pose = pose(x, 0.0);
      input.path.points.push_back(p);
      input.path.left_bound.push_back(pose(x, 6.0).position);
      input.path.right_bound.push_back(pose(x, -6.0).position);
    }
    module->setPreviousModuleOutput(input);
  }

  BehaviorModuleOutput plan(const PredictedObject & o)
  {
    DynamicObstacleAvoidanceTestAccess::targets(*module, o);
    return module->plan();
  }

  std::unique_ptr<ros::NodeHandle> node;
  std::shared_ptr<DynamicAvoidanceParameters> parameters;
  SceneModuleInterface::MarkerInterfaceMap markers;
  std::unique_ptr<DynamicObstacleAvoidanceModule> module;
  BehaviorModuleOutput input;
};

TEST_F(IndependentConstraints, StationaryUnknownKeepsRoadAndExportsIdentity)
{
  auto output = plan(object());
  ASSERT_EQ(output.drivable_area_info.obstacles.size(), 1U);
  const auto & obstacle = output.drivable_area_info.obstacles.front();
  EXPECT_TRUE(obstacle.direct_constraint);
  EXPECT_EQ(obstacle.id, object().object_id);
  EXPECT_EQ(obstacle.hard_margin, 0.1);
  EXPECT_EQ(obstacle.soft_margin, 0.7);
  EXPECT_TRUE(boost::geometry::is_valid(obstacle.poly));
  // Preserve the physical footprint; clearance is carried by explicit margins.
  EXPECT_TRUE(boost::geometry::covered_by(autoware_utils::Point2d(10.0, 2.0), obstacle.poly));
  EXPECT_FALSE(boost::geometry::covered_by(autoware_utils::Point2d(11.7, 2.0), obstacle.poly));
  utils::extractObstaclesFromDrivableArea(output.path, output.drivable_area_info.obstacles);
  EXPECT_EQ(output.path.left_bound, input.path.left_bound);
  EXPECT_EQ(output.path.right_bound, input.path.right_bound);
}

TEST_F(IndependentConstraints, EgoReservationCannotEraseAnObstacleOnThePath)
{
  const auto output = plan(object(10.0, 0.0));
  ASSERT_EQ(output.drivable_area_info.obstacles.size(), 1U);
  EXPECT_TRUE(boost::geometry::covered_by(
    autoware_utils::Point2d(10.0, 0.0), output.drivable_area_info.obstacles.front().poly));
}

TEST_F(IndependentConstraints, PredictionIsIncludedAndRefreshDoesNotLatchOldOccupancy)
{
  auto o = object();
  PredictedPath prediction;
  prediction.confidence = 1.0;
  prediction.time_step.sec = 0;
  prediction.time_step.nanosec = 500000000;
  prediction.path = {pose(10.0, 2.0), pose(12.0, 2.0), pose(14.0, 2.0), pose(30.0, 2.0)};
  o.kinematics.predicted_paths.push_back(prediction);
  auto output = plan(o);
  ASSERT_EQ(output.drivable_area_info.obstacles.size(), 1U);
  EXPECT_TRUE(boost::geometry::covered_by(
    autoware_utils::Point2d(14.0, 2.0), output.drivable_area_info.obstacles.front().poly));
  EXPECT_FALSE(boost::geometry::covered_by(
    autoware_utils::Point2d(30.0, 2.0), output.drivable_area_info.obstacles.front().poly));
  o.kinematics.initial_pose_with_covariance.pose = pose(20.0, 2.0);
  o.kinematics.predicted_paths.front().path = {pose(20.0, 2.0), pose(22.0, 2.0)};
  output = plan(o);
  ASSERT_EQ(output.drivable_area_info.obstacles.size(), 1U);
  EXPECT_FALSE(boost::geometry::covered_by(
    autoware_utils::Point2d(10.0, 2.0), output.drivable_area_info.obstacles.front().poly));
  EXPECT_TRUE(boost::geometry::covered_by(
    autoware_utils::Point2d(22.0, 2.0), output.drivable_area_info.obstacles.front().poly));
}

TEST_F(IndependentConstraints, CurrentFootprintSurvivesConfidenceFiltering)
{
  parameters->threshold_confidence = 0.9;
  auto o = object();
  PredictedPath prediction;
  prediction.confidence = 0.1;
  prediction.path = {pose(25.0, 2.0)};
  o.kinematics.predicted_paths.push_back(prediction);
  const auto output = plan(o);
  ASSERT_EQ(output.drivable_area_info.obstacles.size(), 1U);
  const auto & polygon = output.drivable_area_info.obstacles.front().poly;
  EXPECT_TRUE(boost::geometry::covered_by(autoware_utils::Point2d(10.0, 2.0), polygon));
  EXPECT_FALSE(boost::geometry::covered_by(autoware_utils::Point2d(25.0, 2.0), polygon));
}

TEST_F(IndependentConstraints, StaticOwnerAvoidsDuplicateAndDifferentIdentityIsRetained)
{
  auto static_obstacle = plan(object()).drivable_area_info.obstacles.front();
  static_obstacle.soft_margin = 0.55;
  input.drivable_area_info.obstacles.push_back(static_obstacle);
  module->setPreviousModuleOutput(input);
  auto output = plan(object());
  ASSERT_EQ(output.drivable_area_info.obstacles.size(), 1U);
  EXPECT_EQ(output.drivable_area_info.obstacles.front().soft_margin, 0.55);
  auto other = object(15.0, 2.0);
  other.object_id.uuid[0] = 43;
  output = plan(other);
  EXPECT_EQ(output.drivable_area_info.obstacles.size(), 2U);
}

TEST_F(IndependentConstraints, RegulatedObjectAlsoExportsIndependentSweptOccupancy)
{
  auto o = object();
  o.classification.front().label = autoware_perception_msgs::ObjectClassification::CAR;
  PredictedPath prediction;
  prediction.confidence = 1.0;
  prediction.time_step.sec = 0;
  prediction.time_step.nanosec = 500000000;
  prediction.path = {pose(10.0, 2.0), pose(12.0, 2.0), pose(14.0, 2.0)};
  o.kinematics.predicted_paths.push_back(prediction);
  auto output = plan(o);
  ASSERT_EQ(output.drivable_area_info.obstacles.size(), 1U);
  EXPECT_TRUE(output.drivable_area_info.obstacles.front().direct_constraint);
  EXPECT_TRUE(boost::geometry::covered_by(
    autoware_utils::Point2d(14.0, 2.0), output.drivable_area_info.obstacles.front().poly));
  utils::extractObstaclesFromDrivableArea(output.path, output.drivable_area_info.obstacles);
  EXPECT_EQ(output.path.left_bound, input.path.left_bound);
  EXPECT_EQ(output.path.right_bound, input.path.right_bound);
}
class SpeedOwnership : public IndependentConstraints
{
protected:
  void SetUp() override
  {
    IndependentConstraints::SetUp();
    data = std::make_shared<PlannerData>();
    data->avoidance_object_ownership = std::make_shared<AvoidanceObjectOwnership>();
    nav_msgs::Odometry odom;
    odom.pose.pose.orientation.w = 1.0;
    data->self_odometry = boost::make_shared<nav_msgs::Odometry>(odom);
    module->setData(data);
  }
  BehaviorModuleOutput step(double speed, uint8_t label = autoware_perception_msgs::ObjectClassification::UNKNOWN)
  {
    autoware_perception_msgs::PredictedObjects message;
    message.header.stamp = ros::Time(++stamp);
    auto o = object();
    o.classification.front().label = label;
    o.kinematics.initial_twist_with_covariance.twist.linear.x = speed;
    message.objects.push_back(o);
    data->dynamic_object = boost::make_shared<autoware_perception_msgs::PredictedObjects>(message);
    data->avoidance_object_ownership->update(message);
    module->updateData();
    return module->plan();
  }
  std::shared_ptr<PlannerData> data;
  int stamp{0};
};

TEST_F(SpeedOwnership, ImmediateEntryAndExitBypassLegacyAdmissionCounter)
{
  EXPECT_TRUE(step(0.0).drivable_area_info.obstacles.empty());
  const auto moving = step(0.5);
  ASSERT_EQ(moving.drivable_area_info.obstacles.size(), 1U);
  EXPECT_EQ(moving.drivable_area_info.obstacles.front().hard_margin, 0.1);
  EXPECT_EQ(moving.drivable_area_info.obstacles.front().soft_margin, 0.7);
  EXPECT_EQ(step(0.3).drivable_area_info.obstacles.size(), 1U);
  EXPECT_TRUE(step(0.2).drivable_area_info.obstacles.empty());
  EXPECT_TRUE(step(0.3).drivable_area_info.obstacles.empty());
}

TEST_F(SpeedOwnership, MovingCarExportsPhysicalOccupancyWithoutLegacyPolicyGate)
{
  const auto output = step(0.5, autoware_perception_msgs::ObjectClassification::CAR);
  ASSERT_EQ(output.drivable_area_info.obstacles.size(), 1U);
  EXPECT_EQ(output.path.left_bound, input.path.left_bound);
  EXPECT_EQ(output.path.right_bound, input.path.right_bound);
  EXPECT_FALSE(boost::geometry::covered_by(
    autoware_utils::Point2d(11.7, 2.0), output.drivable_area_info.obstacles.front().poly));
}

TEST_F(SpeedOwnership, DynamicHandoffReplacesStaleStaticConstraintOnce)
{
  DrivableAreaInfo::Obstacle stale;
  stale.id = object().object_id;
  stale.direct_constraint = true;
  input.drivable_area_info.obstacles.push_back(stale);
  module->setPreviousModuleOutput(input);
  auto output = step(0.5);
  ASSERT_EQ(output.drivable_area_info.obstacles.size(), 1U);
  EXPECT_TRUE(boost::geometry::is_valid(output.drivable_area_info.obstacles.front().poly));
  EXPECT_EQ(output.drivable_area_info.obstacles.front().soft_margin, 0.7);
}
}  // namespace
}  // namespace autoware::behavior_path_planner

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
