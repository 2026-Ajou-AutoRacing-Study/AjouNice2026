#include <autoware/motion_velocity_planner_common/polygon_utils.hpp>

#include <gtest/gtest.h>

#include <boost/geometry.hpp>

namespace
{
using autoware::motion_velocity_planner::polygon_utils::getCollisionPoints;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

Polygon2d square(const double center_x, const double center_y, const double half_extent)
{
  Polygon2d polygon;
  polygon.outer() = {
    Point2d{center_x - half_extent, center_y - half_extent},
    Point2d{center_x + half_extent, center_y - half_extent},
    Point2d{center_x + half_extent, center_y + half_extent},
    Point2d{center_x - half_extent, center_y + half_extent},
    Point2d{center_x - half_extent, center_y - half_extent}};
  boost::geometry::correct(polygon);
  return polygon;
}

TEST(PredictedPathCollision, reports_overlap_index_and_stamp)
{
  std::vector<autoware_planning_msgs::TrajectoryPoint> trajectory(2);
  trajectory.at(0).pose.orientation.w = 1.0;
  trajectory.at(1).pose.position.x = 10.0;
  trajectory.at(1).pose.orientation.w = 1.0;
  const std::vector<Polygon2d> trajectory_polygons{square(0.0, 0.0, 1.0), square(10.0, 0.0, 1.0)};

  autoware_perception_msgs::PredictedPath predicted_path;
  predicted_path.time_step.sec = 0;
  predicted_path.time_step.nanosec = 500000000U;
  geometry_msgs::Pose pose;
  pose.position.x = 10.0;
  pose.orientation.w = 1.0;
  predicted_path.path.push_back(pose);

  autoware_perception_msgs::Shape shape;
  shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  shape.dimensions.x = 2.0;
  shape.dimensions.y = 2.0;

  std::vector<size_t> collision_indices;
  const auto points = getCollisionPoints(
    trajectory, trajectory_polygons, ros::Time(10.0), predicted_path, shape, ros::Time(9.0), true,
    collision_indices);

  ASSERT_EQ(points.size(), 1U);
  ASSERT_EQ(collision_indices.size(), 1U);
  EXPECT_EQ(collision_indices.front(), 1U);
  EXPECT_DOUBLE_EQ(points.front().stamp.toSec(), 10.0);
  EXPECT_NEAR(points.front().point.x, 10.0, 1.0);
}

TEST(PredictedPathCollision, ignores_non_overlapping_prediction)
{
  std::vector<autoware_planning_msgs::TrajectoryPoint> trajectory(1);
  trajectory.front().pose.orientation.w = 1.0;
  const std::vector<Polygon2d> trajectory_polygons{square(0.0, 0.0, 1.0)};

  autoware_perception_msgs::PredictedPath predicted_path;
  predicted_path.time_step.sec = 0;
  predicted_path.time_step.nanosec = 500000000U;
  geometry_msgs::Pose pose;
  pose.position.x = 100.0;
  pose.orientation.w = 1.0;
  predicted_path.path.push_back(pose);
  autoware_perception_msgs::Shape shape;
  shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  shape.dimensions.x = 2.0;
  shape.dimensions.y = 2.0;

  std::vector<size_t> collision_indices;
  EXPECT_TRUE(getCollisionPoints(
    trajectory, trajectory_polygons, ros::Time(10.0), predicted_path, shape, ros::Time(9.0), true,
    collision_indices).empty());
  EXPECT_TRUE(collision_indices.empty());
}
}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
