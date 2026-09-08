#include <autoware/costmap_generator/utils/points_to_costmap.hpp>
#include <autoware/costmap_generator/utils/object_map_utils.hpp>
#include <autoware/costmap_generator/utils/objects_to_costmap.hpp>
#include <autoware/costmap_generator/costmap_core.hpp>

#include <gtest/gtest.h>

namespace
{
grid_map::GridMap gridMap()
{
  grid_map::GridMap map;
  map.setFrameId("map");
  map.setGeometry(grid_map::Length(20.0, 20.0), 1.0);
  map.setPosition(grid_map::Position(0.0, 0.0));
  map.add("points", 0.0F);
  return map;
}

size_t occupied(const grid_map::Matrix & matrix)
{
  size_t count{};
  for (int x = 0; x < matrix.rows(); ++x)
    for (int y = 0; y < matrix.cols(); ++y)
      if (matrix(x, y) == 1.0F) ++count;
  return count;
}
}  // namespace

TEST(PointsToCostmap, MarksOnlyInRangePointsAndRejectsOutOfGrid)
{
  pcl::PointCloud<pcl::PointXYZ> points;
  points.emplace_back(0.7F, 1.0F, 1.0F);
  points.emplace_back(1.4F, 2.0F, 2.7F);
  points.emplace_back(2.0F, 2.0F, 9.0F);
  points.emplace_back(11.0F, 11.0F, 1.0F);
  autoware::costmap_generator::PointsToCostmap converter;
  EXPECT_EQ(occupied(converter.makeCostmapFromPoints(5.0, 0.0, 0.0, 1.0, gridMap(), "points", points)), 2U);
}

TEST(PointsToCostmap, PreservesPriorCostForHeightFilteredCellAndClearsUnobservedCells)
{
  auto map = gridMap();
  map["points"].setConstant(1.0F);
  pcl::PointCloud<pcl::PointXYZ> points;
  points.emplace_back(0.0F, 0.0F, -1.0F);
  autoware::costmap_generator::PointsToCostmap converter;
  // Original calculateCostmap clears cells with no points. A cell containing
  // only height-filtered points retains its prior cost; the former reduced
  // ROS1 rasterizer incorrectly erased it along with unobserved cells.
  const auto result = converter.makeCostmapFromPoints(5.0, 0.0, 0.0, 1.0, map, "points", points);
  EXPECT_EQ(occupied(result), 1U);
  EXPECT_FLOAT_EQ(result(10, 10), 1.0F);
  points.clear();
  EXPECT_EQ(occupied(converter.makeCostmapFromPoints(5.0, 0.0, 0.0, 1.0, map, "points", points)), 0U);
}

TEST(ObjectMapUtils, FillsPolygonAndResetsExistingLayer)
{
  geometry_msgs::Polygon rectangle;
  for (const auto point : {std::pair<float, float>{-2.0F, -1.0F}, {-2.0F, 1.0F}, {2.0F, 1.0F}, {2.0F, -1.0F}}) {
    geometry_msgs::Point32 p;
    p.x = point.first;
    p.y = point.second;
    rectangle.points.push_back(p);
  }
  auto map = gridMap();
  map["points"].setConstant(1.0F);
  autoware::costmap_generator::object_map::fill_polygon_areas(map, {rectangle}, "points", 0.0F, 1.0F);
  EXPECT_GT(occupied(map["points"]), 0U);
  EXPECT_LT(occupied(map["points"]), static_cast<size_t>(map["points"].size()));
}

TEST(ObjectsToCostmap, ProjectsBoxScoreAndRejectsUnclassifiedObject)
{
  auto objects = boost::make_shared<autoware_perception_msgs::PredictedObjects>();
  autoware_perception_msgs::PredictedObject box;
  box.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  box.shape.dimensions.x = 4.0; box.shape.dimensions.y = 2.0;
  box.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  autoware_perception_msgs::ObjectClassification label; label.probability = 0.8F;
  box.classification.push_back(label); objects->objects.push_back(box);
  autoware_perception_msgs::PredictedObject empty = box; empty.classification.clear();
  empty.kinematics.initial_pose_with_covariance.pose.position.x = 8.0;
  autoware::costmap_generator::ObjectsToCostmap converter;
  const auto result = converter.makeCostmapFromObjects(gridMap(), 0.0, 1, objects);
  size_t scored{}; for (int x = 0; x < result.rows(); ++x) for (int y = 0; y < result.cols(); ++y)
    if (result(x, y) == 0.8F) ++scored;
  EXPECT_GT(scored, 0U);
  objects->objects.push_back(empty);
  EXPECT_THROW(converter.makeCostmapFromObjects(gridMap(), 0.0, 1, objects), std::invalid_argument);
}

TEST(CostmapCore, InitializesAndCombinesByElementwiseMaximum)
{
  auto map = autoware::costmap_generator::CostmapCore::initialize("map", 4.0, 4.0, 1.0, 0.0, 0.0);
  map.add("points", 0.2F); map.add("objects", 0.5F); map.add("primitives", 0.4F);
  const auto combined = autoware::costmap_generator::CostmapCore::combine(map,"points","objects","primitives","combined",0.0);
  EXPECT_EQ(map.getFrameId(), "map"); EXPECT_FLOAT_EQ(combined(0, 0), 0.5F);
  autoware::costmap_generator::CostmapCore::snapCenterToPose(map, 1.6, -1.6);
  EXPECT_DOUBLE_EQ(map.getPosition().x(), 2.0); EXPECT_DOUBLE_EQ(map.getPosition().y(), -2.0);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "costmap_generator_points_to_costmap_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
