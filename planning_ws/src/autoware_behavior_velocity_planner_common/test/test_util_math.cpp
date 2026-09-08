#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <tf2/utils.h>
#include <autoware/behavior_velocity_planner_common/utilization/trajectory_utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gtest/gtest.h>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

namespace utils = autoware::behavior_velocity_planner::planning_utils;

TEST(OriginalInsertVelocity, InsertsOrReusesPointAndCapsFollowingVelocity)
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  for (int i=0; i<4; ++i) {
    autoware_internal_planning_msgs::PathPointWithLaneId p;
    p.point.pose.position.x = i; p.point.longitudinal_velocity_mps = 8;
    p.lane_ids = {42}; path.points.push_back(p);
  }
  auto stop = path.points[1]; stop.point.pose.position.x = 1.5;
  size_t index = 2;
  utils::insertVelocity(path, stop, 3, index);
  ASSERT_EQ(path.points.size(), 5u);
  EXPECT_EQ(index, 2u);
  EXPECT_DOUBLE_EQ(path.points[2].point.pose.position.x, 1.5);
  EXPECT_EQ(path.points[2].lane_ids, std::vector<int64_t>{42});
  EXPECT_FLOAT_EQ(path.points[1].point.longitudinal_velocity_mps, 8);
  for (size_t i=2; i<path.points.size(); ++i) EXPECT_FLOAT_EQ(path.points[i].point.longitudinal_velocity_mps, 3);
  stop.point.pose.position.x = 2.0001;
  index = 4;
  utils::insertVelocity(path, stop, 0, index);
  EXPECT_EQ(path.points.size(), 5u); EXPECT_EQ(index, 3u);
  EXPECT_FLOAT_EQ(path.points[2].point.longitudinal_velocity_mps, 3);
  EXPECT_FLOAT_EQ(path.points[3].point.longitudinal_velocity_mps, 0);
  EXPECT_FLOAT_EQ(path.points[4].point.longitudinal_velocity_mps, 0);
}

TEST(OriginalRegulatoryLaneIds, ReturnsLaneIdsRatherThanRegulatoryIds)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::LineString3d left{1, {lanelet::Point3d{2,0,1,0}, lanelet::Point3d{3,10,1,0}}};
  lanelet::LineString3d right{4, {lanelet::Point3d{5,0,-1,0}, lanelet::Point3d{6,10,-1,0}}};
  lanelet::Lanelet lane{42, left, right};
  auto light = lanelet::TrafficLight::make(99, {}, {left});
  lane.addRegulatoryElement(light); map->add(lane);
  autoware_internal_planning_msgs::PathWithLaneId path;
  for (int i=0; i<3; ++i) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = i; point.point.pose.orientation.w = 1;
    point.lane_ids = {42}; path.points.push_back(point);
  }
  geometry_msgs::Pose pose; pose.orientation.w = 1;
  EXPECT_EQ(utils::getLaneletIdSetOnPath<lanelet::TrafficLight>(path, map, pose), std::set<int64_t>{42});
  EXPECT_EQ(utils::getRegElemIdSetOnPath<lanelet::TrafficLight>(path, map, pose), std::set<int64_t>{99});
}

TEST(OriginalNoStoppingHelpers, ZeroStartPolygonAndStrictLineCrossing)
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  for (int i=0; i<=4; ++i) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x=i; point.point.pose.orientation.w=1;
    path.points.push_back(point);
  }
  // The original unsigned descending loop underflows here; geometry must
  // retain both boundary samples and terminate at the zero start index.
  const auto polygon=utils::generatePathPolygon(path,0,4,2);
  EXPECT_NEAR(boost::geometry::area(polygon),16,1e-12);
  const auto subset=utils::generatePathPolygon(path,1,4,2);
  EXPECT_NEAR(boost::geometry::area(subset),12,1e-12);
  geometry_msgs::Pose ego,line; ego.position.x=3; line.position.x=2;
  EXPECT_TRUE(utils::isOverLine(path,ego,line));
  EXPECT_FALSE(utils::isOverLine(path,ego,line,1));
  EXPECT_TRUE(utils::isOverLine(path,ego,line,0.999));
  ego.position.x=1; EXPECT_FALSE(utils::isOverLine(path,ego,line));
}

TEST(OriginalDetectionArea, SideSelectionInterpolatedStartAndTerminalPoint)
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  for (int i=0; i<=6; ++i) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = i; point.point.pose.orientation.w = 1;
    point.point.longitudinal_velocity_mps = 10;
    path.points.push_back(point);
  }
  autoware::behavior_velocity_planner::DetectionRange range;
  range.interval = 10; range.max_longitudinal_distance = 10;
  range.max_lateral_distance = 5; range.wheel_tread = 2;
  geometry_msgs::Pose pose; pose.position.x = 0.5; pose.orientation.w = 1;
  autoware::behavior_velocity_planner::Polygons2d polygons;
  ASSERT_TRUE(utils::createDetectionAreaPolygons(polygons, path, pose, 0, range, 1));
  ASSERT_EQ(polygons.size(), 2u);
  EXPECT_NEAR(polygons.front().outer().front().x(), 0.5, 1e-12);
  EXPECT_GT(polygons.front().outer().front().y(), 0);
  EXPECT_LT(polygons.back().outer().front().y(), 0);
  polygons.clear(); range.use_left = false;
  ASSERT_TRUE(utils::createDetectionAreaPolygons(polygons, path, pose, 0, range, 1));
  ASSERT_EQ(polygons.size(), 1u);
  EXPECT_LT(polygons.front().outer().front().y(), 0);
  polygons.clear(); pose.position.x = 6;
  EXPECT_FALSE(utils::createDetectionAreaPolygons(polygons, path, pose, 5, range, 1));
  EXPECT_TRUE(polygons.empty());
}

TEST(OriginalPartitions, SelectsBarrierTypesAndUsesStrictDistanceWithFreshOutput)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  for (int i=0; i<4; ++i) {
    lanelet::LineString3d line{100+i,
      {lanelet::Point3d{200+i*2, double(i*10), -1, 0},
       lanelet::Point3d{201+i*2, double(i*10), 1, 0}}};
    line.attributes()["type"] = std::vector<std::string>{"guard_rail", "fence", "wall", "line_thin"}[i];
    map->add(line);
  }
  autoware::behavior_velocity_planner::BasicPolygons2d partitions;
  utils::getAllPartitionLanelets(map, partitions);
  ASSERT_EQ(partitions.size(), 3u);
  geometry_msgs::Point position; position.x = 0; position.y = 0;
  autoware::behavior_velocity_planner::BasicPolygons2d close = partitions;
  utils::extractClosePartition(position, partitions, close, 10.0);
  ASSERT_EQ(close.size(), 1u);  // barrier exactly at threshold is excluded.
  utils::extractClosePartition(position, {}, close, 100.0);
  EXPECT_TRUE(close.empty());  // never retain previous nearby barriers.
  autoware::behavior_velocity_planner::DetectionRange range;
  EXPECT_TRUE(range.use_left); EXPECT_TRUE(range.use_right);
  EXPECT_DOUBLE_EQ(range.interval, 0.0);
}

namespace
{
autoware_internal_planning_msgs::PathWithLaneId path()
{
  autoware_internal_planning_msgs::PathWithLaneId value;
  for (int i = 0; i < 4; ++i) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = static_cast<double>(i) * 2.0;
    point.point.longitudinal_velocity_mps = 10.0F;
    value.points.push_back(point);
  }
  return value;
}
}  // namespace
int main(int argc, char ** argv) { testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }

TEST(UtilMath, JudgeLineWithAcceleration) { EXPECT_NEAR(26.67, utils::calcJudgeLineDistWithAccLimit(10.0, -3.0, 1.0), 0.01); }
TEST(UtilMath, FromTrajectoryPreservesRestoredPointsBothBoundsAndSourceHeader)
{
  auto value = path();
  for (auto & point : value.points) {
    point.point.pose.orientation.w = 1.0;
    point.lane_ids = {23};
  }
  const auto trajectory = autoware::experimental::trajectory::pretty_build(value.points);
  ASSERT_TRUE(trajectory);
  geometry_msgs::Point left, right;
  left.y = 2.0;
  right.y = -2.0;
  const auto restored = utils::fromTrajectory(*trajectory, {left}, {right});
  EXPECT_EQ(restored.points, trajectory->restore());
  ASSERT_EQ(restored.left_bound.size(), 1U);
  ASSERT_EQ(restored.right_bound.size(), 1U);
  EXPECT_EQ(restored.left_bound.front(), left);
  EXPECT_EQ(restored.right_bound.front(), right);
  EXPECT_TRUE(restored.header.frame_id.empty());
  EXPECT_TRUE(restored.header.stamp.isZero());
}
TEST(UtilMath, JudgeLineWithJerk) { EXPECT_GT(utils::calcJudgeLineDistWithJerkLimit(10.0, 0.0, -3.0, -1.0, 1.0), 0.0); }
TEST(UtilMath, IsAheadOf) {
  geometry_msgs::Pose origin, target;
  origin.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));
  target.position.x = 10.0; EXPECT_TRUE(utils::isAheadOf(target, origin));
  target.position.x = -10.0; EXPECT_FALSE(utils::isAheadOf(target, origin));
}
TEST(UtilMath, DecelerationVelocity) { EXPECT_LT(utils::calcDecelerationVelocityFromDistanceToTarget(-1.0, -3.0, -1.0, 10.0, 100.0), 10.0); }
TEST(UtilPathInsertion, DecelAndStops)
{
  geometry_msgs::Point target; target.x = 3.0;
  auto decel_path = path();
  const auto decel = utils::insertDecelPoint(target, decel_path, 5.0F);
  ASSERT_TRUE(decel.has_value()); EXPECT_DOUBLE_EQ(3.0, decel->position.x);
  EXPECT_LE(decel_path.points.back().point.longitudinal_velocity_mps, 5.0F);
  auto nearest_stop_path = path();
  ASSERT_TRUE(utils::insertStopPoint(target, nearest_stop_path).has_value());
  auto indexed_stop_path = path();
  ASSERT_TRUE(utils::insertStopPoint(target, 1U, indexed_stop_path).has_value());
}
TEST(UtilPathGeometry, ExtendsTwoPointSegmentToBothPathBounds)
{
  const lanelet::BasicLineString2d segment{{0.0, 0.0}, {1.0, 1.0}};
  geometry_msgs::Point lower0, lower1, upper0, upper1;
  lower0.x = -1.0; lower0.y = -1.0; lower1.x = 2.0; lower1.y = -1.0;
  upper0.x = -1.0; upper0.y = 2.0; upper1.x = 2.0; upper1.y = 2.0;
  const auto extended = utils::extendSegmentToBounds(segment, {lower0, lower1}, {upper0, upper1});
  ASSERT_EQ(extended.size(), 2U);
  EXPECT_NEAR(extended.front().x(), -1.0, 1e-6);
  EXPECT_NEAR(extended.front().y(), -1.0, 1e-6);
  EXPECT_NEAR(extended.back().x(), 2.0, 1e-6);
  EXPECT_NEAR(extended.back().y(), 2.0, 1e-6);
  const auto unchanged = utils::extendSegmentToBounds(lanelet::BasicLineString2d{}, {lower0}, {upper0, upper1});
  EXPECT_TRUE(unchanged.empty());
}
TEST(UtilPathLaneIds, KeepsTargetLaneWhenRouteHandlerIsUnavailable)
{
  const std::shared_ptr<autoware::route_handler::RouteHandler> unavailable;
  const auto ids = utils::collectConnectedLaneIds(42, unavailable);
  ASSERT_EQ(ids.size(), 1U);
  EXPECT_EQ(ids.front(), 42);
}
TEST(UtilPathLaneIds, RejectsMissingLaneletMapWithoutDereference)
{
  auto value = path();
  value.points.front().lane_ids = {100};
  geometry_msgs::Pose pose;
  const lanelet::LaneletMapPtr missing_map;
  EXPECT_FALSE(utils::getNearestLaneId(value, missing_map, pose));
  EXPECT_TRUE((utils::getRegElemMapOnPath<lanelet::RegulatoryElement>(value, missing_map, pose)).empty());
  EXPECT_TRUE((utils::getRegElemIdSetOnPath<lanelet::RegulatoryElement>(value, missing_map, pose)).empty());
}
TEST(UtilPathGeometry, AheadPoseInterpolates)
{
  auto value = path();
  value.points.at(1).point.pose.orientation = tf2::toMsg(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  tf2::Quaternion quarter_turn;
  quarter_turn.setRPY(0.0, 0.0, M_PI_2);
  value.points.at(2).point.pose.orientation = tf2::toMsg(quarter_turn);
  const auto result = utils::getAheadPose(0U, 3.0, value);
  EXPECT_DOUBLE_EQ(3.0, result.position.x);
  EXPECT_NEAR(M_PI_4, tf2::getYaw(result.orientation), 1e-12);
}
TEST(UtilPathGeometry, OffsetPointUsesPoseYaw)
{
  geometry_msgs::Pose pose; pose.orientation.w = 1.0;
  const auto result = utils::calculateOffsetPoint2d(pose, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(1.0, result.x()); EXPECT_DOUBLE_EQ(1.0, result.y());
  tf2::Quaternion pitched;
  pitched.setRPY(0.0, M_PI / 3.0, M_PI_2);
  pose.orientation = tf2::toMsg(pitched);
  const auto tilted = utils::calculateOffsetPoint2d(pose, 2.0, 0.0);
  EXPECT_NEAR(tilted.x(), 0.0, 1e-12);
  EXPECT_NEAR(tilted.y(), 1.0, 1e-12);
}
TEST(UtilPathGeometry, PointIndexSelectsForwardSegment)
{
  const auto value = path();
  geometry_msgs::Point target; target.x = 4.5;
  EXPECT_EQ(2U, utils::calcSegmentIndexFromPointIndex(value.points, target, 2U));
}
TEST(UtilPathLaneIds, PreservesFirstEncounterOrderAndSubsequentIds)
{
  auto value = path();
  value.points.at(0).lane_ids = {101, 202};
  value.points.at(1).lane_ids = {202, 303};
  value.points.at(2).lane_ids = {101, 404};
  EXPECT_EQ((std::vector<int64_t>{101, 202, 303, 404}), utils::getSortedLaneIdsFromPath(value));
  EXPECT_EQ((std::vector<int64_t>{202, 303, 404}), utils::getSubsequentLaneIdsSetOnPath(value, 202));
  EXPECT_TRUE(utils::getSubsequentLaneIdsSetOnPath(value, 999).empty());
}
TEST(TrajectoryUtils, RejectsIncompletePlannerData)
{
  autoware_internal_planning_msgs::PathWithLaneId out;
  autoware::behavior_velocity_planner::PlannerData incomplete;
  EXPECT_FALSE(autoware::behavior_velocity_planner::smoothPath(path(), out, incomplete));
}
TEST(IntersectionSourceHelpers, VelocityCapPreservesEarlierAndSlowerPoints)
{
  auto value = path();
  for (auto & point : value.points) point.point.longitudinal_velocity_mps = 7.0;
  value.points.at(2).point.longitudinal_velocity_mps = 1.0;
  utils::setVelocityFromIndex(1, 3.0, &value);
  EXPECT_FLOAT_EQ(value.points.at(0).point.longitudinal_velocity_mps, 7.0f);
  EXPECT_FLOAT_EQ(value.points.at(1).point.longitudinal_velocity_mps, 3.0f);
  EXPECT_FLOAT_EQ(value.points.at(2).point.longitudinal_velocity_mps, 1.0f);
  EXPECT_NO_THROW(utils::setVelocityFromIndex(value.points.size(), 0.0, &value));
}
TEST(IntersectionSourceHelpers, SplineRetainsLaneIdsAndZeroOrderVelocity)
{
  auto input = path();
  for (auto & point : input.points) {
    point.point.pose.orientation.w = 1.0;
    point.lane_ids = {42};
  }
  input.points.at(1).point.longitudinal_velocity_mps = 4.0;
  autoware_internal_planning_msgs::PathWithLaneId output;
  output.header.frame_id = "unchanged";
  EXPECT_FALSE(autoware::behavior_velocity_planner::splineInterpolate({}, 1.0, output, "test"));
  EXPECT_EQ(output.header.frame_id, "unchanged");
  ASSERT_TRUE(autoware::behavior_velocity_planner::splineInterpolate(input, 1.0, output, "test"));
  ASSERT_GE(output.points.size(), 4u);
  EXPECT_NEAR(output.points.at(1).point.pose.position.x, 1.0, 1e-9);
  EXPECT_FLOAT_EQ(output.points.at(1).point.longitudinal_velocity_mps, 10.0f);
  EXPECT_FLOAT_EQ(output.points.at(2).point.longitudinal_velocity_mps, 4.0f);
  for (const auto & point : output.points) EXPECT_EQ(point.lane_ids, (std::vector<int64_t>{42}));
}
TEST(IntersectionSourceHelpers, FailedTrajectoryConversionLeavesPreviousPath)
{
  auto input = path();
  for (auto & point : input.points) point.point.pose.orientation.w = 1.0;
  auto original = autoware::experimental::trajectory::pretty_build(input.points);
  ASSERT_TRUE(original);
  const auto before = original->restore();
  autoware_internal_planning_msgs::PathWithLaneId empty;
  utils::toTrajectory(empty, *original);
  EXPECT_EQ(original->restore(), before);
  auto changed = input;
  for (auto & point : changed.points) point.point.longitudinal_velocity_mps = 2.0;
  utils::toTrajectory(changed, *original);
  for (const auto & point : original->restore()) {
    EXPECT_FLOAT_EQ(point.point.longitudinal_velocity_mps, 2.0f);
  }
}
TEST(IntersectionSourceHelpers, AssociatesSameTurnSuccessorsOfParallelParents)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::Point3d l0(9001,0,2,0), l1(9002,10,2,0), l2(9003,20,2,0);
  lanelet::Point3d c0(9004,0,0,0), c1(9005,10,0,0), c2(9006,20,0,0);
  lanelet::Point3d r0(9007,0,-2,0), r1(9008,10,-2,0), r2(9009,20,-2,0);
  lanelet::LineString3d la(9011,{l0,l1}), lb(9012,{l1,l2});
  lanelet::LineString3d ca(9013,{c0,c1}), cb(9014,{c1,c2});
  lanelet::LineString3d ra(9015,{r0,r1}), rb(9016,{r1,r2});
  for (auto line : {la,lb,ca,cb,ra,rb}) {
    line.attributes()["type"] = "line_thin";
    line.attributes()["subtype"] = "dashed";
  }
  lanelet::Lanelet parent_a(9021,la,ca), parent_b(9022,ca,ra);
  lanelet::Lanelet child_a(9023,lb,cb), child_b(9024,cb,rb);
  for (auto lane : {parent_a,parent_b,child_a,child_b}) {
    lane.attributes()["subtype"] = "road";
    lane.attributes()["location"] = "urban";
    lane.attributes()["one_way"] = "yes";
    map->add(lane);
  }
  child_a.attributes()["turn_direction"] = "left";
  child_b.attributes()["turn_direction"] = "left";
  const auto rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  const lanelet::routing::RoutingGraphPtr graph = lanelet::routing::RoutingGraph::build(*map, *rules);
  ASSERT_FALSE(graph->previous(child_a).empty());
  EXPECT_EQ(utils::getAssociativeIntersectionLanelets(child_a,map,graph),
    (std::set<lanelet::Id>{9023,9024}));
  child_b.attributes()["turn_direction"] = "straight";
  EXPECT_EQ(utils::getAssociativeIntersectionLanelets(child_a,map,graph),
    (std::set<lanelet::Id>{9023}));
  EXPECT_TRUE(utils::getAssociativeIntersectionLanelets(parent_a,map,graph).empty());
  const auto selected = utils::getConstLaneletsFromIds(map, {9024,9021});
  ASSERT_EQ(selected.size(), 2u);
  EXPECT_EQ(selected.front().id(), 9021);
  EXPECT_EQ(selected.back().id(), 9024);
  EXPECT_THROW(utils::getConstLaneletsFromIds(map, {99999}), lanelet::NoSuchPrimitiveError);
}
