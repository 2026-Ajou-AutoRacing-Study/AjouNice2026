#include <autoware/path_generator/route_path_generator_node.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <gtest/gtest.h>

namespace
{
autoware_planning_msgs::LaneletRoute route()
{
  autoware_planning_msgs::LaneletRoute r; r.header.frame_id = "map";
  for (const auto & item : std::vector<std::pair<std::vector<lanelet::Id>, lanelet::Id>>{{{2239,2240,2241,2242},2240},{{2301,2302,2300,2299},2302},{{2244,2245,2246,2247},2245},{{2265,2261,2262,2263},2261}}) {
    autoware_planning_msgs::LaneletSegment s; s.preferred_primitive.id=item.second; s.preferred_primitive.primitive_type="road";
    for (const auto id : item.first) { autoware_planning_msgs::LaneletPrimitive p; p.id=id; p.primitive_type="road"; s.primitives.push_back(p); } r.segments.push_back(s);
  } return r;
}
nav_msgs::Odometry odometry()
{
  nav_msgs::Odometry o; o.pose.pose.position.x=122.81823934652175; o.pose.pose.position.y=264.7931657588342; o.pose.pose.position.z=100.0; o.pose.pose.orientation.z=-0.6922317008371615; o.pose.pose.orientation.w=0.7216753233664642; return o;
}
}  // namespace

TEST(RoutePathGeneratorNode, PublishesPathAfterMapRouteAndOdometry)
{
  ros::NodeHandle node("/path_generator");
  autoware::path_generator::RoutePathGeneratorNode app(node);
  std::vector<autoware_internal_planning_msgs::PathWithLaneId> output;
  auto sub=node.subscribe<autoware_internal_planning_msgs::PathWithLaneId>("output/path",1,[&](const auto & m){output.push_back(*m);});
  auto map_pub=node.advertise<autoware_map_msgs::LaneletMapBin>("input/vector_map",1,true);
  auto route_pub=node.advertise<autoware_planning_msgs::LaneletRoute>("input/route",1,true);
  auto odom_pub=node.advertise<nav_msgs::Odometry>("input/odometry",1,true);
  const auto map=autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(AW149_PATH_GENERATOR_TEST_MAP);
  ASSERT_TRUE(map); map_pub.publish(autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map)); route_pub.publish(route()); odom_pub.publish(odometry());
  const auto deadline=ros::WallTime::now()+ros::WallDuration(5.0);
  while (output.empty() && ros::WallTime::now()<deadline) { ros::spinOnce(); ros::WallDuration(0.01).sleep(); }
  ASSERT_EQ(output.size(),1U); EXPECT_EQ(output.front().header.frame_id,"map"); EXPECT_GT(output.front().header.stamp, ros::Time(0)); EXPECT_FALSE(output.front().points.empty()); EXPECT_FALSE(output.front().points.front().lane_ids.empty());
}
int main(int argc,char ** argv) { ros::init(argc,argv,"path_generator_node_oracle"); ::testing::InitGoogleTest(&argc,argv); return RUN_ALL_TESTS(); }
