#include "autoware/path_generator/node.hpp"
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>
#include <ros/package.h>
#include <gtest/gtest.h>
#include <cmath>
namespace autoware::path_generator {
TEST(FullNativePathGenerator, PollingInputsAndOwnedTimerPublishActualPath)
{
  ros::NodeHandle node("/full_path_generator_test");
  PathGenerator app(node);
  std::vector<PathWithLaneId> outputs;
  auto sub=node.subscribe<PathWithLaneId>("output/path",1,[&](const auto & message){outputs.push_back(*message);});
  auto map_pub=node.advertise<LaneletMapBin>("input/vector_map",1,true);
  auto route_pub=node.advertise<LaneletRoute>("input/route",1,true);
  auto odom_pub=node.advertise<Odometry>("input/odometry",1,true);
  auto spin_until=[&](double seconds,bool wait_for_path){
    const auto end=ros::WallTime::now()+ros::WallDuration(seconds);
    while(ros::WallTime::now()<end && (!wait_for_path || outputs.empty())) {
      ros::spinOnce(); ros::WallDuration(0.01).sleep();
    }
  };
  spin_until(0.3,false); EXPECT_TRUE(outputs.empty());
  const auto map=autoware::test_utils::make_map_bin_msg(ros::package::getPath("autoware_lanelet2_utils")+"/sample_map/vm_01_10-12/dense_centerline/lanelet2_map.osm");
  const auto route=autoware::test_utils::parse<std::optional<LaneletRoute>>(
    autoware::test_utils::get_absolute_path_to_route("autoware_path_generator","dense_centerline_route.yaml"));
  ASSERT_TRUE(route); ASSERT_FALSE(route->segments.empty());
  Odometry odometry; odometry.header.frame_id="map"; odometry.pose.pose=route->start_pose;
  map_pub.publish(map); route_pub.publish(*route); odom_pub.publish(odometry);
  spin_until(5.0,true); ASSERT_FALSE(outputs.empty());
  const auto & path=outputs.back();
  EXPECT_EQ(path.header.frame_id,route->header.frame_id); EXPECT_GT(path.header.stamp,ros::Time(0));
  EXPECT_FALSE(path.left_bound.empty()); EXPECT_FALSE(path.right_bound.empty()); ASSERT_FALSE(path.points.empty());
  for(const auto & point:path.points) {
    EXPECT_FALSE(point.lane_ids.empty());
    EXPECT_TRUE(std::isfinite(point.point.pose.position.x));
    EXPECT_TRUE(std::isfinite(point.point.longitudinal_velocity_mps));
    EXPECT_GE(point.point.longitudinal_velocity_mps,0);
  }
  // Invalid empty route is ignored by original take_data; last route remains usable.
  outputs.clear(); route_pub.publish(LaneletRoute{}); spin_until(2.0,true);
  EXPECT_FALSE(outputs.empty());
}
TEST(FullNativePathGenerator, SchemaRejectsInvalidUpdateAndRetainsLastValidParams)
{
  ros::NodeHandle node("/full_path_generator_test");
  ::path_generator::ParamListener listener(node);
  const auto original=listener.get_params();
  node.setParam("planning_hz",0.0);
  EXPECT_DOUBLE_EQ(listener.get_params().planning_hz,original.planning_hz);
  EXPECT_THROW(::path_generator::ParamListener invalid(node),std::exception);
  node.setParam("planning_hz",original.planning_hz);
  node.setParam("path_length/forward",original.path_length.forward+1.0);
  EXPECT_DOUBLE_EQ(listener.get_params().path_length.forward,original.path_length.forward+1.0);
  node.setParam("path_length/forward",original.path_length.forward);
}
}
