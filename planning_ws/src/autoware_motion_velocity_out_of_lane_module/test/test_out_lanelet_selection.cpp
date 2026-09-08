#include "../src/lanelets_selection.hpp"
#include "../src/out_of_lane_collisions.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_perception_msgs/PredictedObject.h>
#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <ros/time.h>

#include <boost/geometry/algorithms/correct.hpp>

namespace
{
lanelet::Lanelet make_road_lanelet(const lanelet::Id id, const double lower_y, const double upper_y)
{
  lanelet::LineString3d left(
    id * 10 + 1,
    {lanelet::Point3d(id * 10 + 2, 0.0, upper_y, 0.0),
     lanelet::Point3d(id * 10 + 3, 40.0, upper_y, 0.0)});
  lanelet::LineString3d right(
    id * 10 + 4,
    {lanelet::Point3d(id * 10 + 5, 0.0, lower_y, 0.0),
     lanelet::Point3d(id * 10 + 6, 40.0, lower_y, 0.0)});
  lanelet::Lanelet lanelet(id, left, right);
  lanelet.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  return lanelet;
}

autoware_utils::Polygon2d make_footprint()
{
  autoware_utils::Polygon2d footprint;
  footprint.outer().emplace_back(10.0, 1.5);
  footprint.outer().emplace_back(30.0, 1.5);
  footprint.outer().emplace_back(30.0, 2.5);
  footprint.outer().emplace_back(10.0, 2.5);
  footprint.outer().emplace_back(10.0, 1.5);
  boost::geometry::correct(footprint);
  return footprint;
}

lanelet::LaneletMapPtr make_two_lane_map()
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(make_road_lanelet(1, -2.0, 2.0));
  map->add(make_road_lanelet(2, 2.0, 4.0));
  return map;
}
}  // namespace

TEST(OutOfLaneSelection, SelectsOnlyNonTrajectoryAndNonIgnoredRoadOverlap)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(make_road_lanelet(1, -2.0, 2.0));
  map->add(make_road_lanelet(2, 2.0, 4.0));
  map->add(make_road_lanelet(3, -4.0, -2.0));

  autoware_utils::MultiPolygon2d footprints;
  footprints.push_back(make_footprint());
  const lanelet::ConstLanelets trajectory_lanelets{map->laneletLayer.get(1)};
  const lanelet::ConstLanelets ignored_lanelets{map->laneletLayer.get(3)};
  const auto selected = autoware::motion_velocity_planner::out_of_lane::calculate_out_lanelets(
    map->laneletLayer, footprints, trajectory_lanelets, ignored_lanelets);

  ASSERT_EQ(selected.size(), 1U);
  EXPECT_EQ(selected.front().id(), 2);
}

TEST(OutOfLaneCollisionDecision, MarksTtcConflictForAvoidance)
{
  using autoware::motion_velocity_planner::out_of_lane::CollisionTime;
  using autoware::motion_velocity_planner::out_of_lane::OutOfLaneData;
  using autoware::motion_velocity_planner::out_of_lane::OutOfLanePoint;
  using autoware::motion_velocity_planner::out_of_lane::PlannerParam;

  OutOfLaneData data;
  OutOfLanePoint point;
  point.trajectory_index = 0U;
  CollisionTime collision;
  collision.collision_time = 1.0;
  point.collision_times.insert(collision);
  data.outside_points.push_back(point);

  autoware_planning_msgs::TrajectoryPoint trajectory_point;
  trajectory_point.time_from_start.sec = 1;
  trajectory_point.time_from_start.nanosec = 0;
  const std::vector<autoware_planning_msgs::TrajectoryPoint> trajectory{trajectory_point};
  PlannerParam params;
  params.mode = "ttc";
  params.ttc_threshold = 1.0;
  params.ttc_release_threshold = 3.0;

  autoware::motion_velocity_planner::out_of_lane::calculate_collisions_to_avoid(
    data, trajectory, params);

  ASSERT_TRUE(data.outside_points.front().ttc.has_value());
  EXPECT_DOUBLE_EQ(*data.outside_points.front().ttc, 0.0);
  EXPECT_TRUE(data.outside_points.front().to_avoid);
}

TEST(OutOfLaneCollisionDecision, ConvertsObjectOverlapIntoTtcAvoidance)
{
  using autoware::motion_velocity_planner::out_of_lane::OutOfLaneData;
  using autoware::motion_velocity_planner::out_of_lane::OutOfLanePoint;
  using autoware::motion_velocity_planner::out_of_lane::PlannerParam;
  const auto map = make_two_lane_map();
  auto map_message = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  map_message.version_map_format = "1.1";
  map_message.version_map = "out-of-lane-test";
  const autoware::route_handler::RouteHandler handler(map_message);
  ASSERT_TRUE(handler.isMapMsgReady());

  OutOfLaneData data;
  OutOfLanePoint point;
  point.trajectory_index = 0U;
  point.out_overlaps.push_back(make_footprint());
  point.overlapped_lanelets.push_back(map->laneletLayer.get(2));
  data.outside_points.push_back(point);
  autoware::motion_velocity_planner::out_of_lane::prepare_out_of_lane_areas_rtree(data);

  autoware_perception_msgs::PredictedObject object;
  object.object_id.uuid[0] = 149U;
  object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 1.0;
  object.shape.dimensions.y = 1.0;
  object.shape.dimensions.z = 1.0;
  autoware_perception_msgs::PredictedPath path;
  path.time_step.sec = 1;
  path.time_step.nanosec = 0;
  geometry_msgs::Pose pose;
  pose.position.x = 20.0;
  pose.position.y = 2.0;
  pose.orientation.w = 1.0;
  path.path.push_back(pose);
  object.kinematics.predicted_paths.push_back(path);

  autoware::motion_velocity_planner::out_of_lane::calculate_object_path_time_collisions(
    data, 0U, object, handler, false);
  ASSERT_EQ(data.outside_points.front().collision_times.size(), 1U);
  EXPECT_DOUBLE_EQ(data.outside_points.front().collision_times.begin()->collision_time, 0.0);

  autoware_planning_msgs::TrajectoryPoint trajectory_point;
  trajectory_point.time_from_start.sec = 0;
  trajectory_point.time_from_start.nanosec = 0;
  PlannerParam params;
  params.mode = "ttc";
  params.ttc_threshold = 1.0;
  params.ttc_release_threshold = 3.0;
  autoware::motion_velocity_planner::out_of_lane::calculate_collisions_to_avoid(
    data, {trajectory_point}, params);
  EXPECT_TRUE(data.outside_points.front().to_avoid);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
