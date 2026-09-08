#include <autoware/lanelet2_utils/conversion.hpp>

#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <lanelet2_core/LaneletMap.h>

#include <atomic>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  if (argc != 2 && argc != 3) {
    std::cerr << "usage: run_out_host_map_probe <valid-lanelet-map.osm> [--crossing-object|--map-only]"
              << std::endl;
    return 64;
  }
  const bool crossing_object = argc == 3 && std::string(argv[2]) == "--crossing-object";
  const bool map_only = argc == 3 && std::string(argv[2]) == "--map-only";
  if (argc == 3 && !crossing_object && !map_only) {
    std::cerr << "unsupported probe mode: " << argv[2] << std::endl;
    return 64;
  }
  ros::init(argc, argv, "aw149_run_out_host_map_probe");
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(argv[1]);
  if (!map || map->laneletLayer.empty()) {
    std::cerr << "map loader returned no lanelets" << std::endl;
    return 2;
  }
  const auto map_message = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  if (map_message.data.empty()) {
    std::cerr << "LaneletMapBin serialization produced no data" << std::endl;
    return 3;
  }

  ros::NodeHandle node;
  std::atomic<size_t> output_count{0};
  std::atomic<bool> output_velocity_changed{false};
  std::mutex output_mutex;
  autoware_planning_msgs::Trajectory output;
  const auto output_subscriber = node.subscribe<autoware_planning_msgs::Trajectory>(
    "/motion_velocity_planner/output/trajectory", 1,
    [&output_count, &output_velocity_changed, &output_mutex, &output](
      const autoware_planning_msgs::TrajectoryConstPtr & message) {
      {
        std::lock_guard<std::mutex> lock(output_mutex);
        output = *message;
      }
      for (const auto & point : message->points) {
        if (point.longitudinal_velocity_mps != 5.0F) {
          output_velocity_changed = true;
          break;
        }
      }
      ++output_count;
    });
  const auto trajectory_publisher = node.advertise<autoware_planning_msgs::Trajectory>(
    "/motion_velocity_planner/input/trajectory", 1, true);
  const auto objects_publisher = node.advertise<autoware_perception_msgs::PredictedObjects>(
    "/motion_velocity_planner/input/predicted_objects", 1, true);
  const auto odometry_publisher = node.advertise<nav_msgs::Odometry>(
    "/motion_velocity_planner/input/odometry", 1, true);
  const auto acceleration_publisher = node.advertise<geometry_msgs::AccelWithCovarianceStamped>(
    "/motion_velocity_planner/input/acceleration", 1, true);
  const auto map_publisher = node.advertise<autoware_map_msgs::LaneletMapBin>(
    "/motion_velocity_planner/input/lanelet_map", 1, true);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  if (map_only) {
    const auto map_deadline = ros::WallTime::now() + ros::WallDuration(5.0);
    while (ros::ok() && ros::WallTime::now() < map_deadline) {
      map_publisher.publish(map_message);
      ros::WallDuration(0.05).sleep();
    }
    std::cout << "R031_LANELET_MAP_ONLY_TRANSPORT_PASS serialized_bytes="
              << map_message.data.size() << std::endl;
    spinner.stop();
    return 0;
  }

  const auto connection_deadline = ros::WallTime::now() + ros::WallDuration(5.0);
  while (
    ros::ok() && ros::WallTime::now() < connection_deadline &&
    (trajectory_publisher.getNumSubscribers() == 0 || objects_publisher.getNumSubscribers() == 0 ||
     odometry_publisher.getNumSubscribers() == 0 || acceleration_publisher.getNumSubscribers() == 0 ||
     map_publisher.getNumSubscribers() == 0)) {
    ros::WallDuration(0.05).sleep();
  }
  if (
    trajectory_publisher.getNumSubscribers() == 0 || objects_publisher.getNumSubscribers() == 0 ||
    odometry_publisher.getNumSubscribers() == 0 || acceleration_publisher.getNumSubscribers() == 0 ||
    map_publisher.getNumSubscribers() == 0) {
    std::cerr << "MVP host did not connect all RunOut inputs" << std::endl;
    return 4;
  }

  nav_msgs::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = 5.0;
  geometry_msgs::AccelWithCovarianceStamped acceleration;
  autoware_perception_msgs::PredictedObjects objects;
  objects.header.frame_id = "map";
  if (crossing_object) {
    autoware_perception_msgs::PredictedObject object;
    object.object_id.uuid[0] = 1;
    object.existence_probability = 1.0F;
    object.classification.emplace_back();
    object.classification.back().label = autoware_perception_msgs::ObjectClassification::PEDESTRIAN;
    object.classification.back().probability = 1.0F;
    object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
    object.shape.dimensions.x = 0.6;
    object.shape.dimensions.y = 0.6;
    object.shape.dimensions.z = 1.7;
    object.kinematics.initial_pose_with_covariance.pose.position.x = 10.0;
    object.kinematics.initial_pose_with_covariance.pose.position.y = -10.0;
    object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
    // The module's stopped-object classifier uses the longitudinal component.
    // Keep it nonzero while the predicted path itself crosses laterally.
    object.kinematics.initial_twist_with_covariance.twist.linear.x = 1.0;
    object.kinematics.predicted_paths.emplace_back();
    auto & predicted_path = object.kinematics.predicted_paths.back();
    predicted_path.confidence = 1.0F;
    predicted_path.time_step.sec = 1;
    for (int index = 0; index < 4; ++index) {
      geometry_msgs::Pose pose;
      pose.position.x = 10.0;
      pose.position.y = -10.0 + static_cast<double>(index * 5);
      pose.orientation.w = 1.0;
      predicted_path.path.push_back(pose);
    }
    objects.objects.push_back(object);
  }
  autoware_planning_msgs::Trajectory trajectory;
  trajectory.header.frame_id = "map";
  for (int index = 0; index < 4; ++index) {
    autoware_planning_msgs::TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(index * 5);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 5.0F;
    point.time_from_start.sec = index;
    trajectory.points.push_back(point);
  }
  map_publisher.publish(map_message);
  objects_publisher.publish(objects);
  odometry_publisher.publish(odometry);
  acceleration_publisher.publish(acceleration);
  const auto output_deadline = ros::WallTime::now() + ros::WallDuration(5.0);
  while (
    ros::ok() && ros::WallTime::now() < output_deadline &&
    (output_count.load() == 0 || (crossing_object && !output_velocity_changed.load()))) {
    trajectory_publisher.publish(trajectory);
    ros::WallDuration(0.05).sleep();
  }
  autoware_planning_msgs::Trajectory output_snapshot;
  {
    std::lock_guard<std::mutex> lock(output_mutex);
    output_snapshot = output;
  }
  const bool invalid_empty_scene_shape =
    !crossing_object && output_snapshot.points.size() != trajectory.points.size();
  const bool invalid_crossing_shape = crossing_object && output_snapshot.points.empty();
  if (
    output_count.load() == 0 || output_snapshot.header.frame_id != trajectory.header.frame_id ||
    invalid_empty_scene_shape || invalid_crossing_shape) {
    std::cerr << "RunOut host did not publish a valid constrained trajectory: outputs="
              << output_count.load() << " frame='" << output_snapshot.header.frame_id
              << "' points=" << output_snapshot.points.size() << std::endl;
    return 5;
  }
  for (const auto & point : output_snapshot.points) {
    if (!crossing_object && point.longitudinal_velocity_mps != 5.0F) {
      std::cerr << "RunOut changed empty-scene velocity" << std::endl;
      return 6;
    }
  }
  if (crossing_object && !output_velocity_changed.load()) {
    std::cerr << "RunOut did not constrain the synchronized crossing-object trajectory" << std::endl;
    return 7;
  }
  std::cout << (crossing_object ? "R030_RUN_OUT_CROSSING_OBJECT_PASS" : "R027_RUN_OUT_VALID_MAP_EMPTY_SCENE_PASS")
            << " points=" << output_snapshot.points.size()
            << " serialized_bytes=" << map_message.data.size() << std::endl;
  spinner.stop();
  (void)output_subscriber;
  return 0;
}
