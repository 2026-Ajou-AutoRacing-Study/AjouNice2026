#include "../src/out_of_lane_module.hpp"
#include "../src/calculate_slowdown_points.hpp"
#include "../src/filter_predicted_objects.hpp"
#include "../src/footprint.hpp"
#include "../src/lanelets_selection.hpp"
#include "../src/out_of_lane_collisions.hpp"

#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <autoware_perception_msgs/PredictedObject.h>
#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <ros/ros.h>

namespace
{
lanelet::Lanelet make_road_lanelet(lanelet::Id id, double lower_y, double upper_y)
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

void set_required_parameters(ros::NodeHandle & node)
{
  node.setParam("out_of_lane/mode", "ttc");
  node.setParam("out_of_lane/skip_if_already_overlapping", false);
  node.setParam("out_of_lane/max_arc_length", 100.0);
  node.setParam("out_of_lane/threshold/time_threshold", 5.0);
  node.setParam("out_of_lane/ttc/threshold", 1.0);
  node.setParam("out_of_lane/ttc/release_threshold", 3.0);
  node.setParam("out_of_lane/objects/minimum_velocity", 0.5);
  node.setParam("out_of_lane/objects/predicted_path_min_confidence", 0.1);
  node.setParam("out_of_lane/objects/cut_predicted_paths_beyond_red_lights", false);
  node.setParam("out_of_lane/objects/ignore_behind_ego", false);
  node.setParam("out_of_lane/objects/validate_predicted_paths_on_lanelets", false);
  node.setParam("out_of_lane/objects/extra_width", 1.0);
  node.setParam("out_of_lane/action/use_map_stop_lines", false);
  node.setParam("out_of_lane/action/precision", 0.1);
  node.setParam("out_of_lane/action/longitudinal_distance_buffer", 1.5);
  node.setParam("out_of_lane/action/lateral_distance_buffer", 1.0);
  node.setParam("out_of_lane/action/min_on_duration", 0.0);
  node.setParam("out_of_lane/action/min_off_duration", 1.0);
  node.setParam("out_of_lane/action/update_distance_th", 5.0);
  node.setParam("out_of_lane/action/slowdown/velocity", 2.0);
  node.setParam("out_of_lane/action/stop/distance_threshold", 15.0);
  node.setParam("out_of_lane/ego/extra_front_offset", 0.0);
  node.setParam("out_of_lane/ego/extra_rear_offset", 0.0);
  node.setParam("out_of_lane/ego/extra_right_offset", 0.0);
  node.setParam("out_of_lane/ego/extra_left_offset", 0.0);
  node.setParam("vehicle_info/max_longitudinal_offset_m", 3.74);
  node.setParam("vehicle_info/rear_overhang_m", 1.03);
  node.setParam("vehicle_info/vehicle_width_m", 1.83);
  node.setParam("vehicle_info/max_steer_angle", 0.70);
  node.setParam("vehicle_info/wheel_base_m", 2.74);
  node.setParam("vehicle_info/wheel_tread_m", 1.63);
  node.setParam("vehicle_info/front_overhang_m", 1.0);
  node.setParam("vehicle_info/left_overhang_m", 0.1);
  node.setParam("vehicle_info/right_overhang_m", 0.1);
  node.setParam("vehicle_info/vehicle_height_m", 2.5);
  node.setParam("wheel_radius", 0.39);
  node.setParam("wheel_width", 0.42);
  node.setParam("wheel_base", 2.74);
  node.setParam("wheel_tread", 1.63);
  node.setParam("front_overhang", 1.0);
  node.setParam("rear_overhang", 1.03);
  node.setParam("left_overhang", 0.1);
  node.setParam("right_overhang", 0.1);
  node.setParam("vehicle_height", 2.5);
  node.setParam("max_steer_angle", 0.70);
}

autoware_planning_msgs::TrajectoryPoint trajectory_point(double x, double y)
{
  autoware_planning_msgs::TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.orientation.w = 1.0;
  point.longitudinal_velocity_mps = 8.0;
  const ros::Duration time_from_start(x / 8.0);
  point.time_from_start.sec = time_from_start.sec;
  point.time_from_start.nanosec = time_from_start.nsec;
  return point;
}
}  // namespace

TEST(OutOfLanePlan, ConflictObjectProducesSlowdownOrStopOutput)
{
  ros::NodeHandle node("~");
  set_required_parameters(node);
  const auto module_vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  EXPECT_DOUBLE_EQ(module_vehicle_info.max_longitudinal_offset_m, 3.74);
  EXPECT_DOUBLE_EQ(module_vehicle_info.min_longitudinal_offset_m, -1.03);
  EXPECT_DOUBLE_EQ(module_vehicle_info.max_lateral_offset_m, 0.915);
  EXPECT_DOUBLE_EQ(module_vehicle_info.min_lateral_offset_m, -0.915);
  autoware::motion_velocity_planner::OutOfLaneModule module;
  module.init(node, "out_of_lane");

  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(make_road_lanelet(1, -2.0, 2.0));
  map->add(make_road_lanelet(2, 2.0, 4.0));
  auto data = std::make_shared<autoware::motion_velocity_planner::PlannerData>();
  data->route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map);
  data->current_odometry.pose.pose.position.y = 1.9;
  data->current_odometry.pose.pose.orientation.w = 1.0;
  data->current_odometry.twist.twist.linear.x = 8.0;
  data->vehicle_info_.wheel_base_m = 2.74;
  data->vehicle_info_.front_overhang_m = 1.0;
  data->vehicle_info_.rear_overhang_m = 1.03;
  data->vehicle_info_.wheel_tread_m = 1.63;
  data->vehicle_info_.left_overhang_m = 0.1;
  data->vehicle_info_.right_overhang_m = 0.1;
  data->vehicle_info_.max_longitudinal_offset_m = 3.74;
  data->vehicle_info_.min_longitudinal_offset_m = -1.03;
  data->vehicle_info_.max_lateral_offset_m = 0.915;
  data->vehicle_info_.min_lateral_offset_m = -0.915;

  autoware_perception_msgs::PredictedObject object;
  object.object_id.uuid[0] = 149U;
  object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 1.0;
  object.shape.dimensions.y = 1.0;
  object.shape.dimensions.z = 1.0;
  object.kinematics.initial_pose_with_covariance.pose.position.x = 0.0;
  object.kinematics.initial_pose_with_covariance.pose.position.y = 4.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  autoware_perception_msgs::PredictedPath path;
  const ros::Duration time_step(2.5);
  path.time_step.sec = time_step.sec;
  path.time_step.nanosec = time_step.nsec;
  path.confidence = 1.0;
  geometry_msgs::Pose first = object.kinematics.initial_pose_with_covariance.pose;
  geometry_msgs::Pose second = first;
  second.position.x = 20.0;
  second.position.y = 2.5;
  path.path = {first, second};
  object.kinematics.predicted_paths.push_back(path);
  data->objects.push_back(std::make_shared<autoware::motion_velocity_planner::PlannerData::Object>(object));

  const std::vector<autoware_planning_msgs::TrajectoryPoint> trajectory{
    trajectory_point(0.0, 1.9), trajectory_point(10.0, 1.9), trajectory_point(20.0, 1.9),
    trajectory_point(30.0, 1.9)};

  autoware::motion_velocity_planner::out_of_lane::PlannerParam params{};
  params.mode = "ttc";
  params.skip_if_already_overlapping = false;
  params.max_arc_length = 100.0;
  params.time_threshold = 5.0;
  params.ttc_threshold = 1.0;
  params.ttc_release_threshold = 3.0;
  params.objects_min_vel = 0.5;
  params.objects_min_confidence = 0.1;
  params.objects_extra_width = 1.0;
  params.objects_cut_predicted_paths_beyond_red_lights = false;
  params.objects_ignore_behind_ego = false;
  params.validate_predicted_paths_on_lanelets = false;
  params.precision = 0.1;
  params.lon_dist_buffer = 1.5;
  params.lat_dist_buffer = 1.0;
  params.slow_velocity = 2.0;
  params.stop_dist_threshold = 15.0;
  params.min_on_duration = 0.0;
  params.min_off_duration = 1.0;
  params.update_distance_th = 5.0;
  params.use_map_stop_lines = false;
  params.front_offset = data->vehicle_info_.max_longitudinal_offset_m;
  params.rear_offset = data->vehicle_info_.min_longitudinal_offset_m;
  params.left_offset = data->vehicle_info_.max_lateral_offset_m;
  params.right_offset = data->vehicle_info_.min_lateral_offset_m;

  autoware::motion_velocity_planner::out_of_lane::EgoData ego;
  ego.pose = data->current_odometry.pose.pose;
  ego.velocity = data->current_odometry.twist.twist.linear.x;
  ego.trajectory_points = trajectory;
  ego.current_footprint =
    autoware::motion_velocity_planner::out_of_lane::calculate_current_ego_footprint(ego, params, true);
  ego.trajectory_footprints =
    autoware::motion_velocity_planner::out_of_lane::calculate_trajectory_footprints(ego, params);
  autoware::motion_velocity_planner::out_of_lane::calculate_out_lanelet_rtree(
    ego, *data->route_handler, params);
  EXPECT_FALSE(ego.out_lanelets.empty());
  auto stage_data = autoware::motion_velocity_planner::out_of_lane::OutOfLaneData{};
  stage_data.outside_points =
    autoware::motion_velocity_planner::out_of_lane::calculate_out_of_lane_points(ego);
  autoware::motion_velocity_planner::out_of_lane::prepare_out_of_lane_areas_rtree(stage_data);
  EXPECT_FALSE(stage_data.outside_points.empty());
  const auto filtered =
    autoware::motion_velocity_planner::out_of_lane::filter_predicted_objects(*data, ego, params);
  ASSERT_EQ(filtered.objects.size(), 1U);
  ASSERT_EQ(filtered.objects.front().kinematics.predicted_paths.size(), 1U);
  autoware::motion_velocity_planner::out_of_lane::calculate_objects_time_collisions(
    stage_data, filtered.objects, *data->route_handler, params);
  autoware::motion_velocity_planner::out_of_lane::calculate_collisions_to_avoid(
    stage_data, trajectory, params, false);
  EXPECT_TRUE(std::any_of(
    stage_data.outside_points.begin(), stage_data.outside_points.end(),
    [](const auto & point) { return point.to_avoid; }));
  const auto avoided = std::find_if(
    stage_data.outside_points.cbegin(), stage_data.outside_points.cend(),
    [](const auto & point) { return point.to_avoid; });
  ASSERT_NE(avoided, stage_data.outside_points.cend());
  EXPECT_TRUE(autoware::motion_velocity_planner::out_of_lane::calculate_slowdown_pose(
                ego, *avoided, params)
                .has_value());

  const auto result = module.plan(trajectory, trajectory, data);
  EXPECT_TRUE(!result.stop_points.empty() || !result.slowdown_intervals.empty());
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "out_of_lane_plan_oracle");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
