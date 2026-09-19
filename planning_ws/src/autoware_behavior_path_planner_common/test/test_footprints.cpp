// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/footprints.hpp>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>

namespace
{
using autoware::behavior_path_planner::drivable_area_expansion::DrivableAreaExpansionParameters;
using autoware::behavior_path_planner::drivable_area_expansion::Point2d;
using autoware::behavior_path_planner::drivable_area_expansion::Polygon2d;

constexpr auto epsilon = 1e-6;

geometry_msgs::Pose makePose(const double x, const double y, const double yaw)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

TEST(FootprintTest, TranslatePolygon)
{
  using autoware::behavior_path_planner::drivable_area_expansion::translate_polygon;
  Polygon2d polygon;
  polygon.outer() = {
    Point2d{0.0, 0.0}, Point2d{1.0, 0.0}, Point2d{1.0, 1.0}, Point2d{0.0, 1.0}, Point2d{0.0, 0.0}};

  const auto translated = translate_polygon(polygon, 1.0, 2.0);
  ASSERT_EQ(translated.outer().size(), 5U);
  EXPECT_NEAR(translated.outer().at(0).x(), 1.0, epsilon);
  EXPECT_NEAR(translated.outer().at(0).y(), 2.0, epsilon);
  EXPECT_NEAR(translated.outer().at(2).x(), 2.0, epsilon);
  EXPECT_NEAR(translated.outer().at(2).y(), 3.0, epsilon);
}

TEST(FootprintTest, CreateFootprintWithTranslationAndRotation)
{
  using autoware::behavior_path_planner::drivable_area_expansion::create_footprint;
  Polygon2d base;
  base.outer() = {
    Point2d{1.0, 1.0}, Point2d{2.0, 1.0}, Point2d{2.0, 2.0}, Point2d{1.0, 2.0}, Point2d{1.0, 1.0}};

  const auto translated = create_footprint(makePose(1.0, 2.0, 0.0), base);
  ASSERT_EQ(translated.outer().size(), 5U);
  EXPECT_NEAR(translated.outer().at(0).x(), 2.0, epsilon);
  EXPECT_NEAR(translated.outer().at(0).y(), 3.0, epsilon);

  const auto rotated = create_footprint(makePose(1.0, 2.0, M_PI_2), base);
  ASSERT_EQ(rotated.outer().size(), 5U);
  EXPECT_NEAR(rotated.outer().at(0).x(), 0.0, epsilon);
  EXPECT_NEAR(rotated.outer().at(0).y(), 3.0, epsilon);
  EXPECT_NEAR(rotated.outer().at(2).x(), -1.0, epsilon);
  EXPECT_NEAR(rotated.outer().at(2).y(), 4.0, epsilon);
}

TEST(FootprintTest, CreateObjectFootprintsFromDynamicPredictedPath)
{
  using autoware::behavior_path_planner::drivable_area_expansion::create_object_footprints;
  autoware_perception_msgs::PredictedObjects objects;
  autoware_perception_msgs::PredictedObject object;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;
  autoware_perception_msgs::PredictedPath path;
  path.path.push_back(makePose(0.0, 0.0, 0.0));
  object.kinematics.predicted_paths.push_back(path);
  objects.objects.push_back(object);

  DrivableAreaExpansionParameters params;
  params.object_exclusion.front_offset = 0.5;
  params.object_exclusion.rear_offset = 0.5;
  params.object_exclusion.left_offset = 0.5;
  params.object_exclusion.right_offset = 0.5;
  EXPECT_TRUE(create_object_footprints(objects, params).empty());

  params.object_exclusion.exclude_dynamic = true;
  const auto footprints = create_object_footprints(objects, params);
  ASSERT_EQ(footprints.size(), 1U);
  ASSERT_EQ(footprints.front().outer().size(), 5U);
  EXPECT_NEAR(footprints.front().outer().at(0).x(), 2.5, epsilon);
  EXPECT_NEAR(footprints.front().outer().at(0).y(), 1.5, epsilon);
  EXPECT_NEAR(footprints.front().outer().at(2).x(), -2.5, epsilon);
  EXPECT_NEAR(footprints.front().outer().at(2).y(), -1.5, epsilon);
}
}  // namespace
