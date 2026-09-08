#include <autoware/motion_velocity_planner_common/utils.hpp>

#include <gtest/gtest.h>

namespace
{
std::vector<autoware_planning_msgs::TrajectoryPoint> straightTrajectory()
{
  std::vector<autoware_planning_msgs::TrajectoryPoint> points(11);
  for (size_t i = 0; i < points.size(); ++i) {
    auto & point = points.at(i);
    point.pose.position.x = static_cast<double>(i * 2U);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 3.0F;
  }
  return points;
}
}  // namespace

TEST(MotionVelocityUtils, DecimatesFromEgoAndExtendsGoalWithSourceVelocity)
{
  geometry_msgs::Pose ego_pose;
  ego_pose.position.x = 5.0;
  ego_pose.orientation.w = 1.0;
  const auto output = autoware::motion_velocity_planner::utils::decimate_trajectory_points_from_ego(
    straightTrajectory(), ego_pose, 5.0, 1.57, 4.0, 4.0);

  ASSERT_GE(output.size(), 2U);
  EXPECT_GE(output.front().pose.position.x, 4.0);
  EXPECT_GT(output.back().pose.position.x, 20.0);
  EXPECT_FLOAT_EQ(output.back().longitudinal_velocity_mps, 3.0F);
  EXPECT_NEAR(output.back().pose.orientation.w, 1.0, 1e-6);
}

TEST(MotionVelocityUtils, LeavesShortTrajectoryUnchangedRatherThanResamplingIt)
{
  std::vector<autoware_planning_msgs::TrajectoryPoint> one_point(1);
  one_point.front().pose.position.x = 7.0;
  geometry_msgs::Pose ego_pose;
  ego_pose.orientation.w = 1.0;
  const auto output = autoware::motion_velocity_planner::utils::decimate_trajectory_points_from_ego(
    one_point, ego_pose, 5.0, 1.57, 2.0, 5.0);
  ASSERT_EQ(output.size(), 1U);
  EXPECT_DOUBLE_EQ(output.front().pose.position.x, 7.0);
}

TEST(MotionVelocityUtils, SelectsNearestPointAtSignedLongitudinalOffset)
{
  const auto points = straightTrajectory();
  EXPECT_EQ(
    autoware::motion_velocity_planner::utils::get_index_with_longitudinal_offset(points, 2.8, 0U),
    1U);
  EXPECT_EQ(
    autoware::motion_velocity_planner::utils::get_index_with_longitudinal_offset(points, -2.8, 3U),
    2U);
}

TEST(MotionVelocityUtils, OriginalShapeRadiusAndUnsupportedShapeContract)
{
  using namespace autoware::motion_velocity_planner;
  autoware_perception_msgs::Shape shape;
  shape.type=shape.BOUNDING_BOX;shape.dimensions.x=6.0;shape.dimensions.y=8.0;
  EXPECT_DOUBLE_EQ(utils::calc_object_possible_max_dist_from_center(shape),5.0);
  shape.type=shape.CYLINDER;
  EXPECT_DOUBLE_EQ(utils::calc_object_possible_max_dist_from_center(shape),3.0);
  shape.type=shape.POLYGON;
  geometry_msgs::Point32 vertex;vertex.x=3.0;vertex.y=4.0;
  shape.footprint.points.push_back(vertex);
  EXPECT_DOUBLE_EQ(utils::calc_object_possible_max_dist_from_center(shape),5.0);
  shape.type=255;
  EXPECT_THROW(utils::calc_object_possible_max_dist_from_center(shape),std::logic_error);
}

TEST(MotionVelocityUtils, OriginalPointAndConcatenationContracts)
{
  using namespace autoware::motion_velocity_planner;
  pcl::PointXYZ point;point.x=1.0;point.y=2.0;point.z=3.0;
  const auto spatial=utils::to_geometry_point(point);
  EXPECT_DOUBLE_EQ(spatial.x,1.0);EXPECT_DOUBLE_EQ(spatial.y,2.0);EXPECT_DOUBLE_EQ(spatial.z,3.0);
  const auto planar=utils::to_geometry_point(autoware_utils_geometry::Point2d{4.0,5.0});
  EXPECT_DOUBLE_EQ(planar.x,4.0);EXPECT_DOUBLE_EQ(planar.y,5.0);EXPECT_DOUBLE_EQ(planar.z,0.0);
  EXPECT_EQ(utils::concat_vectors(std::vector<int>{1,2},std::vector<int>{3}), (std::vector<int>{1,2,3}));
  EXPECT_EQ(utils::concat_vectors(std::vector<int>{},std::vector<int>{3}), (std::vector<int>{3}));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
