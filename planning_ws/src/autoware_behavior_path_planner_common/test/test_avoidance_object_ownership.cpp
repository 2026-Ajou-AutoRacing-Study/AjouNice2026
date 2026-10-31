#include <autoware/behavior_path_planner_common/avoidance_object_ownership.hpp>
#include <gtest/gtest.h>
#include <limits>
#include <vector>

using autoware::behavior_path_planner::AvoidanceObjectOwnership;
namespace
{
void observe(AvoidanceObjectOwnership & policy, double t, double vx, double vy = 0.0, double x = 0.0)
{
  autoware_perception_msgs::PredictedObjects message;
  message.header.stamp = ros::Time(t);
  message.objects.resize(1);
  auto & object = message.objects.front();
  object.kinematics.initial_twist_with_covariance.twist.linear.x = vx;
  object.kinematics.initial_twist_with_covariance.twist.linear.y = vy;
  object.kinematics.initial_pose_with_covariance.pose.position.x = x;
  policy.update(message);
}
unique_identifier_msgs::UUID id;
using Classification = autoware_perception_msgs::ObjectClassification;
Classification classification(uint8_t label, float probability = 1.0F)
{
  Classification result;
  result.label = label;
  result.probability = probability;
  return result;
}
void observeClass(
  AvoidanceObjectOwnership & policy, double t, double speed,
  const std::vector<Classification> & labels)
{
  autoware_perception_msgs::PredictedObjects message;
  message.header.stamp = ros::Time(t);
  message.objects.resize(1);
  message.objects.front().classification = labels;
  message.objects.front().kinematics.initial_twist_with_covariance.twist.linear.x = speed;
  policy.update(message);
}
TEST(AvoidanceObjectOwnership, ExactBoundariesSwitchOnFirstObservation)
{
  AvoidanceObjectOwnership policy;
  observe(policy, 1.0, 0.0);
  EXPECT_EQ(policy.isDynamic(id), false);
  observe(policy, 1.01, 0.5);
  EXPECT_EQ(policy.isDynamic(id), true);
  observe(policy, 1.02, 0.2);
  EXPECT_EQ(policy.isDynamic(id), false);
}
TEST(AvoidanceObjectOwnership, BandRetainsEachPreviousOwnerWithoutElapsedTime)
{
  AvoidanceObjectOwnership stationary, moving;
  observe(stationary, 1.0, 0.3);
  observe(moving, 1.0, 0.5);
  for (double t : {1.01, 100.0, 10000.0}) {
    observe(stationary, t, 0.3);
    observe(moving, t, 0.3);
    EXPECT_EQ(stationary.isDynamic(id), false);
    EXPECT_EQ(moving.isDynamic(id), true);
  }
}
TEST(AvoidanceObjectOwnership, UsesPlanarMagnitudeIncludingReverseAndCrossingMotion)
{
  AvoidanceObjectOwnership policy;
  observe(policy, 1.0, 0.0, -0.5);
  EXPECT_EQ(policy.isDynamic(id), true);
  observe(policy, 2.0, 0.0);
  observe(policy, 3.0, -0.5);
  EXPECT_EQ(policy.isDynamic(id), true);
}
TEST(AvoidanceObjectOwnership, PositionJitterCannotChangeOwnership)
{
  AvoidanceObjectOwnership policy;
  observe(policy, 1.0, 0.0);
  observe(policy, 1.01, 0.0, 0.0, 10.0);
  EXPECT_EQ(policy.isDynamic(id), false);
}
TEST(AvoidanceObjectOwnership, SameInputStampDoesNotAdvanceDecision)
{
  AvoidanceObjectOwnership policy;
  observe(policy, 1.0, 0.0);
  observe(policy, 1.0, 1.0);
  EXPECT_EQ(policy.isDynamic(id), false);
  observe(policy, 1.01, 1.0);
  EXPECT_EQ(policy.isDynamic(id), true);
}
TEST(AvoidanceObjectOwnership, MissingDetectionRetainsOnlyOwnerAndClearResetsIt)
{
  AvoidanceObjectOwnership policy;
  observe(policy, 1.0, 1.0);
  autoware_perception_msgs::PredictedObjects empty;
  empty.header.stamp = ros::Time(2.0);
  policy.update(empty);
  observe(policy, 3.0, 0.3);
  EXPECT_EQ(policy.isDynamic(id), true);
  policy.clear();
  observe(policy, 4.0, 0.3);
  EXPECT_EQ(policy.isDynamic(id), false);
}
TEST(AvoidanceObjectOwnership, RewindStartsFreshAndInvalidVelocityKeepsCollisionOwnership)
{
  AvoidanceObjectOwnership policy;
  observe(policy, 2.0, 1.0);
  observe(policy, 1.0, 0.3);
  EXPECT_EQ(policy.isDynamic(id), false);
  observe(policy, 1.1, std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(policy.isDynamic(id), true);
}
TEST(AvoidanceObjectOwnership, ThresholdValidation)
{
  EXPECT_THROW(AvoidanceObjectOwnership(0.2, 0.5), std::invalid_argument);
  EXPECT_THROW(AvoidanceObjectOwnership(0.5, -0.1), std::invalid_argument);
  EXPECT_THROW(AvoidanceObjectOwnership(0.5, 0.2, 0.2, 0.2), std::invalid_argument);
  EXPECT_THROW(AvoidanceObjectOwnership(0.5, 0.2, 0.7, -0.1), std::invalid_argument);
  EXPECT_THROW(AvoidanceObjectOwnership(0.5, 0.2, std::numeric_limits<double>::quiet_NaN(), 0.2), std::invalid_argument);
  EXPECT_THROW(AvoidanceObjectOwnership(0.5, 0.2, 0.7, std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(AvoidanceObjectOwnership, LegacyConstructorsKeepPedestrianGenericThresholds)
{
  AvoidanceObjectOwnership default_policy, two_argument_policy(0.5, 0.2);
  for (auto * policy : {&default_policy, &two_argument_policy}) {
    observeClass(*policy, 1.0, 0.5, {classification(Classification::PEDESTRIAN)});
    EXPECT_EQ(policy->isDynamic(id), true);
  }
}

TEST(AvoidanceObjectOwnership, PedestrianSpecificBoundariesAreImmediateAndHysteretic)
{
  AvoidanceObjectOwnership policy(0.5, 0.2, 0.7, 0.2);
  const auto pedestrian = std::vector<Classification>{classification(Classification::PEDESTRIAN)};
  observeClass(policy, 1.0, 0.699, pedestrian);
  EXPECT_EQ(policy.isDynamic(id), false);
  observeClass(policy, 1.01, 0.7, pedestrian);
  EXPECT_EQ(policy.isDynamic(id), true);
  observeClass(policy, 1.02, 0.201, pedestrian);
  EXPECT_EQ(policy.isDynamic(id), true);
  observeClass(policy, 1.03, 0.2, pedestrian);
  EXPECT_EQ(policy.isDynamic(id), false);
  observeClass(policy, 1000.0, 0.6, pedestrian);
  EXPECT_EQ(policy.isDynamic(id), false);
}

TEST(AvoidanceObjectOwnership, AllOtherValidClassesKeepGenericThresholds)
{
  for (uint8_t label = Classification::UNKNOWN; label < Classification::PEDESTRIAN; ++label) {
    SCOPED_TRACE(static_cast<int>(label));
    AvoidanceObjectOwnership policy(0.5, 0.2, 0.7, 0.2);
    observeClass(policy, 1.0, 0.5, {classification(label)});
    EXPECT_EQ(policy.isDynamic(id), true);
    observeClass(policy, 1.1, 0.2, {classification(label)});
    EXPECT_EQ(policy.isDynamic(id), false);
  }
}

TEST(AvoidanceObjectOwnership, OnlyValidUniqueMaximumConfidencePedestrianUsesSlowerEntry)
{
  AvoidanceObjectOwnership policy(0.5, 0.2, 0.7, 0.2);
  observeClass(policy, 1.0, 0.6,
    {classification(Classification::CAR, 0.2F), classification(Classification::PEDESTRIAN, 0.8F)});
  EXPECT_EQ(policy.isDynamic(id), false);
  const std::vector<std::vector<Classification>> ambiguous{
    {}, {classification(Classification::PEDESTRIAN, 0.0F)},
    {classification(Classification::PEDESTRIAN, 0.5F), classification(Classification::CAR, 0.5F)},
    {classification(Classification::PEDESTRIAN, 0.5F), classification(Classification::PEDESTRIAN, 0.5F)},
    {classification(Classification::PEDESTRIAN, std::numeric_limits<float>::quiet_NaN())},
    {classification(Classification::PEDESTRIAN, -0.1F)},
    {classification(Classification::PEDESTRIAN, 1.1F)},
    {classification(Classification::PEDESTRIAN, 0.9F), classification(255, 0.1F)}};
  for (const auto & labels : ambiguous) {
    policy.clear();
    observeClass(policy, 2.0, 0.5, labels);
    EXPECT_EQ(policy.isDynamic(id), true);
  }
}

TEST(AvoidanceObjectOwnership, ClassChangesApplyNewHysteresisWithoutArbitraryDynamicRelease)
{
  AvoidanceObjectOwnership policy(0.5, 0.2, 0.7, 0.2);
  observeClass(policy, 1.0, 0.6, {classification(Classification::CAR)});
  EXPECT_EQ(policy.isDynamic(id), true);
  observeClass(policy, 1.1, 0.6, {classification(Classification::PEDESTRIAN)});
  EXPECT_EQ(policy.isDynamic(id), true);
  observeClass(policy, 1.2, 0.2, {classification(Classification::PEDESTRIAN)});
  EXPECT_EQ(policy.isDynamic(id), false);
  observeClass(policy, 1.3, 0.6, {classification(Classification::PEDESTRIAN)});
  EXPECT_EQ(policy.isDynamic(id), false);
  observeClass(policy, 1.4, 0.6, {classification(Classification::UNKNOWN)});
  EXPECT_EQ(policy.isDynamic(id), true);
}

TEST(AvoidanceObjectOwnership, PedestrianDuplicateRewindAndInvalidSpeedKeepOldSafetySemantics)
{
  AvoidanceObjectOwnership policy(0.5, 0.2, 0.7, 0.2);
  const auto pedestrian = std::vector<Classification>{classification(Classification::PEDESTRIAN)};
  observeClass(policy, 2.0, 0.0, pedestrian);
  observeClass(policy, 2.0, 0.7, pedestrian);
  EXPECT_EQ(policy.isDynamic(id), false);
  observeClass(policy, 2.1, 0.7, pedestrian);
  EXPECT_EQ(policy.isDynamic(id), true);
  observeClass(policy, 1.0, 0.6, pedestrian);
  EXPECT_EQ(policy.isDynamic(id), false);
  observeClass(policy, 1.1, std::numeric_limits<double>::quiet_NaN(), pedestrian);
  EXPECT_EQ(policy.isDynamic(id), true);
}

TEST(AvoidanceObjectOwnership, OwnerClassificationDoesNotMutateInputVelocityShapeOrPaths)
{
  AvoidanceObjectOwnership policy(0.5, 0.2, 0.7, 0.2);
  autoware_perception_msgs::PredictedObjects message;
  message.header.stamp = ros::Time(1.0);
  message.objects.resize(1);
  auto & object = message.objects.front();
  object.classification = {classification(Classification::PEDESTRIAN)};
  object.kinematics.initial_twist_with_covariance.twist.linear.x = 0.6;
  object.kinematics.initial_pose_with_covariance.pose.position.x = 42.0;
  object.shape.dimensions.x = 0.4;
  object.kinematics.predicted_paths.resize(1);
  object.kinematics.predicted_paths.front().path.resize(31);
  policy.update(message);
  EXPECT_EQ(policy.isDynamic(id), false);
  EXPECT_DOUBLE_EQ(object.kinematics.initial_twist_with_covariance.twist.linear.x, 0.6);
  EXPECT_DOUBLE_EQ(object.kinematics.initial_pose_with_covariance.pose.position.x, 42.0);
  EXPECT_DOUBLE_EQ(object.shape.dimensions.x, 0.4);
  EXPECT_EQ(object.kinematics.predicted_paths.front().path.size(), 31U);
}
}  // namespace
