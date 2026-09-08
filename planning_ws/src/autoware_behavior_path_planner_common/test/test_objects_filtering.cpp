#include <autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp>

#include <gtest/gtest.h>

namespace safety = autoware::behavior_path_planner::utils::path_safety_checker;

namespace
{
autoware_perception_msgs::PredictedObject object_with_speed(
  const double speed, const uint8_t label, const float probability)
{
  autoware_perception_msgs::PredictedObject object;
  object.kinematics.initial_twist_with_covariance.twist.linear.x = speed;
  autoware_perception_msgs::ObjectClassification classification;
  classification.label = label;
  classification.probability = probability;
  object.classification.push_back(classification);
  return object;
}
}  // namespace

TEST(ObjectsFilteringSourceAdapter, PreservesVelocityClassAndConfidenceSemantics)
{
  autoware_perception_msgs::PredictedObjects objects;
  objects.objects.push_back(object_with_speed(1.0, autoware_perception_msgs::ObjectClassification::CAR, 1.0F));
  objects.objects.push_back(object_with_speed(4.0, autoware_perception_msgs::ObjectClassification::PEDESTRIAN, 1.0F));

  EXPECT_EQ(safety::filterObjectsByVelocity(objects, 2.0, false).objects.size(), 1U);
  EXPECT_EQ(safety::filterObjectsByVelocity(objects, 2.0, 5.0).objects.size(), 1U);

  safety::ObjectTypesToCheck types;
  types.check_pedestrian = false;
  auto by_class = objects;
  safety::filterObjectsByClass(by_class, types);
  ASSERT_EQ(by_class.objects.size(), 1U);
  EXPECT_EQ(by_class.objects.front().classification.front().label,
            autoware_perception_msgs::ObjectClassification::CAR);

  std::vector<autoware_perception_msgs::PredictedPath> paths(3);
  paths.at(0).confidence = 0.2F;
  paths.at(1).confidence = 0.8F;
  paths.at(2).confidence = 0.8F;
  const auto highest = safety::get_highest_confidence_paths(paths);
  ASSERT_EQ(highest.size(), 2U);
  EXPECT_FLOAT_EQ(highest.at(0).confidence, 0.8F);
  EXPECT_FLOAT_EQ(highest.at(1).confidence, 0.8F);
}
