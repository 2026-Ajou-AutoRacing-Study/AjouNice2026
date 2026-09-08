#include <autoware/behavior_velocity_walkway_module/ros1_support.hpp>
#include "scene_walkway.hpp"
#include "experimental/scene_walkway.hpp"
#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <pluginlib/class_loader.hpp>
#include <gtest/gtest.h>
#include <type_traits>
namespace bvp = autoware::behavior_velocity_planner;
template<class Interface> void plugin(const std::string & base, const std::string & prefix)
{
  ros::NodeHandle node("/full_walkway_test");
  pluginlib::ClassLoader<Interface> loader("autoware_behavior_velocity_planner_common", base);
  auto instance = loader.createUniqueInstance(prefix + "WalkwayModulePlugin");
  instance->init(node);
  EXPECT_STREQ(instance->getModuleName(), "walkway");
  const auto required = instance->getRequiredSubscriptions();
  EXPECT_FALSE(required.traffic_signals); EXPECT_FALSE(required.predicted_objects);
  EXPECT_FALSE(required.no_ground_pointcloud); EXPECT_FALSE(required.occupancy_grid_map);
}
TEST(FullWalkway, NormalPlugin) { plugin<bvp::PluginInterface>("autoware::behavior_velocity_planner::PluginInterface", "autoware::behavior_velocity_planner::"); }
TEST(FullWalkway, ExperimentalPlugin) { plugin<bvp::experimental::PluginInterface>("autoware::behavior_velocity_planner::experimental::PluginInterface", "autoware::behavior_velocity_planner::experimental::"); }

template<class Module> void lifecycle()
{
  ros::NodeHandle node("/full_walkway_test");
  auto factors = std::make_shared<autoware::planning_factor_interface::PlanningFactorInterface>(&node, "walkway_test");
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::LineString3d left{1,{lanelet::Point3d{2,10,2,0},lanelet::Point3d{3,12,2,0}}};
  lanelet::LineString3d right{4,{lanelet::Point3d{5,10,-2,0},lanelet::Point3d{6,12,-2,0}}};
  map->add(lanelet::Lanelet{42,left,right});
  typename Module::PlannerParam params; params.stop_distance_from_crosswalk = 1; params.stop_duration = 1;
  Module module(42, map, params, false, node, "walkway_test", std::make_shared<bvp::WalkwayRos1Clock>(), nullptr, factors);
  auto data = std::make_shared<bvp::PlannerData>();
  auto ego = std::make_shared<geometry_msgs::PoseStamped>(); ego->pose.orientation.w=1;
  data->current_odometry = ego; data->vehicle_info_.max_longitudinal_offset_m = 1;
  geometry_msgs::TwistStamped velocity; velocity.header.stamp=ros::Time::now(); velocity.twist.linear.x=5;
  data->velocity_buffer.push_back(velocity);
  auto fresh = [] {
    autoware_internal_planning_msgs::PathWithLaneId value;
    for(int i=0; i<=20; ++i) {
      autoware_internal_planning_msgs::PathPointWithLaneId p;
      p.point.pose.position.x=i; p.point.pose.orientation.w=1;
      p.point.longitudinal_velocity_mps=5; p.lane_ids={42}; value.points.push_back(p);
    }
    return value;
  };
  auto modify = [&](autoware_internal_planning_msgs::PathWithLaneId & path) {
    if constexpr (std::is_same_v<Module,bvp::WalkwayModule>) {
      module.setPlannerData(data); return module.modifyPathVelocity(&path);
    } else {
      auto trajectory = autoware::experimental::trajectory::pretty_build(path.points);
      if (!trajectory) return false;
      bool result = module.modifyPathVelocity(*trajectory, path.left_bound, path.right_bound, *data);
      path.points = trajectory->restore(); return result;
    }
  };
  auto input=fresh(); ASSERT_TRUE(modify(input));
  const auto stop = std::find_if(input.points.begin(), input.points.end(), [](const auto & p){return p.point.longitudinal_velocity_mps==0;});
  ASSERT_NE(stop,input.points.end()); EXPECT_NEAR(stop->point.pose.position.x,8,1e-6);
  EXPECT_FALSE(module.createVirtualWalls().empty());
  EXPECT_FALSE(module.createDebugMarkerArray().markers.empty());
  ego->pose.position.x=8;
  data->velocity_buffer.front().twist.linear.x=0;
  data->velocity_buffer.front().header.stamp=ros::Time::now()-ros::Duration(2);
  input=fresh(); ASSERT_TRUE(modify(input)); // APPROACH -> STOP
  input=fresh(); ASSERT_TRUE(modify(input)); // STOP -> SURPASSED
  input=fresh(); ASSERT_TRUE(modify(input));
  for(const auto & p:input.points) EXPECT_FLOAT_EQ(p.point.longitudinal_velocity_mps,5);
}
TEST(FullWalkway, NormalApproachStopAndSurpass) { lifecycle<bvp::WalkwayModule>(); }
TEST(FullWalkway, ExperimentalApproachStopAndSurpass) { lifecycle<bvp::experimental::WalkwayModule>(); }
int main(int argc,char ** argv) {
  ros::init(argc,argv,"full_walkway"); ros::NodeHandle owner;
  testing::InitGoogleTest(&argc,argv); return RUN_ALL_TESTS();
}
