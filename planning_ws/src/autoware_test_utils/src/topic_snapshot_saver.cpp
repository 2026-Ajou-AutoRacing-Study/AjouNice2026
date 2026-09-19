// Copyright 2024 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0

#include "autoware_test_utils/autoware_test_utils.hpp"
#include "autoware_test_utils/mock_data_parser.hpp"

#include <ros/ros.h>

#include <autoware_adapi_v1_msgs/OperationModeState.h>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_perception_msgs/TrackedObjects.h>
#include <autoware_perception_msgs/TrafficLightGroupArray.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>

namespace
{
using MessageType = std::variant<
  nav_msgs::Odometry, geometry_msgs::AccelWithCovarianceStamped,
  autoware_perception_msgs::PredictedObjects, autoware_adapi_v1_msgs::OperationModeState,
  autoware_planning_msgs::LaneletRoute, autoware_perception_msgs::TrafficLightGroupArray,
  autoware_perception_msgs::TrackedObjects, autoware_internal_planning_msgs::PathWithLaneId>;

std::mutex g_mutex;

std::optional<size_t> get_topic_index(const std::string & name)
{
  static const std::unordered_map<std::string, size_t> indices{
    {"Odometry", 0}, {"AccelWithCovarianceStamped", 1}, {"PredictedObjects", 2},
    {"OperationModeState", 3}, {"LaneletRoute", 4}, {"TrafficLightGroupArray", 5},
    {"TrackedObjects", 6}, {"PathWithLaneId", 7}};
  const auto it = indices.find(name);
  return it == indices.end() ? std::nullopt : std::optional<size_t>{it->second};
}

template <size_t I>
using RosMsgType = std::variant_alternative_t<I, MessageType>;

template <typename TimeT>
YAML::Node time_yaml(const TimeT & value)
{
  YAML::Node node;
  node["sec"] = value.sec;
  node["nanosec"] = value.nanosec;
  return node;
}

YAML::Node header_yaml(const std_msgs::Header & value)
{
  YAML::Node node;
  node["stamp"]["sec"] = value.stamp.sec;
  node["stamp"]["nanosec"] = value.stamp.nsec;
  node["frame_id"] = value.frame_id;
  return node;
}

template <typename PointT>
YAML::Node point_yaml(const PointT & value)
{
  YAML::Node node;
  node["x"] = value.x;
  node["y"] = value.y;
  node["z"] = value.z;
  return node;
}

YAML::Node pose_yaml(const geometry_msgs::Pose & value)
{
  YAML::Node node;
  node["position"] = point_yaml(value.position);
  node["orientation"]["x"] = value.orientation.x;
  node["orientation"]["y"] = value.orientation.y;
  node["orientation"]["z"] = value.orientation.z;
  node["orientation"]["w"] = value.orientation.w;
  return node;
}

template <typename CovarianceT>
YAML::Node covariance_yaml(const CovarianceT & value)
{
  YAML::Node node;
  for (const auto entry : value) node.push_back(entry);
  return node;
}

YAML::Node pose_covariance_yaml(const geometry_msgs::PoseWithCovariance & value)
{
  YAML::Node node;
  node["pose"] = pose_yaml(value.pose);
  node["covariance"] = covariance_yaml(value.covariance);
  return node;
}

template <typename VectorT>
YAML::Node vector_yaml(const VectorT & value)
{
  return point_yaml(value);
}

YAML::Node twist_yaml(const geometry_msgs::Twist & value)
{
  YAML::Node node;
  node["linear"] = vector_yaml(value.linear);
  node["angular"] = vector_yaml(value.angular);
  return node;
}

YAML::Node twist_covariance_yaml(const geometry_msgs::TwistWithCovariance & value)
{
  YAML::Node node;
  node["twist"] = twist_yaml(value.twist);
  node["covariance"] = covariance_yaml(value.covariance);
  return node;
}

YAML::Node accel_yaml(const geometry_msgs::Accel & value)
{
  YAML::Node node;
  node["linear"] = vector_yaml(value.linear);
  node["angular"] = vector_yaml(value.angular);
  return node;
}

YAML::Node accel_covariance_yaml(const geometry_msgs::AccelWithCovariance & value)
{
  YAML::Node node;
  node["accel"] = accel_yaml(value.accel);
  node["covariance"] = covariance_yaml(value.covariance);
  return node;
}

YAML::Node uuid_yaml(const unique_identifier_msgs::UUID & value)
{
  YAML::Node node;
  for (const auto entry : value.uuid) node["uuid"].push_back(entry);
  return node;
}

YAML::Node shape_yaml(const autoware_perception_msgs::Shape & value)
{
  YAML::Node node;
  node["type"] = value.type;
  for (const auto & point : value.footprint.points) node["footprint"]["points"].push_back(point_yaml(point));
  node["dimensions"] = point_yaml(value.dimensions);
  return node;
}

YAML::Node classification_yaml(const autoware_perception_msgs::ObjectClassification & value)
{
  YAML::Node node;
  node["label"] = value.label;
  node["probability"] = value.probability;
  return node;
}

YAML::Node predicted_path_yaml(const autoware_perception_msgs::PredictedPath & value)
{
  YAML::Node node;
  for (const auto & pose : value.path) node["path"].push_back(pose_yaml(pose));
  node["time_step"] = time_yaml(value.time_step);
  node["confidence"] = value.confidence;
  return node;
}

YAML::Node predicted_object_yaml(const autoware_perception_msgs::PredictedObject & value)
{
  YAML::Node node;
  node["object_id"] = uuid_yaml(value.object_id);
  node["existence_probability"] = value.existence_probability;
  for (const auto & item : value.classification) node["classification"].push_back(classification_yaml(item));
  const auto & kinematics = value.kinematics;
  node["kinematics"]["initial_pose_with_covariance"] = pose_covariance_yaml(kinematics.initial_pose_with_covariance);
  node["kinematics"]["initial_twist_with_covariance"] = twist_covariance_yaml(kinematics.initial_twist_with_covariance);
  node["kinematics"]["initial_acceleration_with_covariance"] = accel_covariance_yaml(kinematics.initial_acceleration_with_covariance);
  for (const auto & path : kinematics.predicted_paths) node["kinematics"]["predicted_paths"].push_back(predicted_path_yaml(path));
  node["shape"] = shape_yaml(value.shape);
  return node;
}

YAML::Node tracked_object_yaml(const autoware_perception_msgs::TrackedObject & value)
{
  YAML::Node node;
  node["object_id"] = uuid_yaml(value.object_id);
  node["existence_probability"] = value.existence_probability;
  for (const auto & item : value.classification) node["classification"].push_back(classification_yaml(item));
  const auto & kinematics = value.kinematics;
  node["kinematics"]["pose_with_covariance"] = pose_covariance_yaml(kinematics.pose_with_covariance);
  node["kinematics"]["twist_with_covariance"] = twist_covariance_yaml(kinematics.twist_with_covariance);
  node["kinematics"]["acceleration_with_covariance"] = accel_covariance_yaml(kinematics.acceleration_with_covariance);
  node["kinematics"]["orientation_availability"] = kinematics.orientation_availability;
  node["kinematics"]["is_stationary"] = kinematics.is_stationary;
  node["shape"] = shape_yaml(value.shape);
  return node;
}

YAML::Node path_point_yaml(const autoware_internal_planning_msgs::PathPointWithLaneId & value)
{
  YAML::Node node;
  node["point"]["pose"] = pose_yaml(value.point.pose);
  node["point"]["longitudinal_velocity_mps"] = value.point.longitudinal_velocity_mps;
  node["point"]["lateral_velocity_mps"] = value.point.lateral_velocity_mps;
  node["point"]["heading_rate_rps"] = value.point.heading_rate_rps;
  node["point"]["is_final"] = value.point.is_final;
  for (const auto lane_id : value.lane_ids) node["lane_ids"].push_back(lane_id);
  return node;
}

YAML::Node route_yaml(const autoware_planning_msgs::LaneletRoute & value)
{
  YAML::Node node;
  node["start_pose"] = pose_yaml(value.start_pose);
  node["goal_pose"] = pose_yaml(value.goal_pose);
  for (const auto & segment : value.segments) {
    YAML::Node item;
    item["preferred_primitive"]["id"] = segment.preferred_primitive.id;
    item["preferred_primitive"]["primitive_type"] = segment.preferred_primitive.primitive_type;
    for (const auto & primitive : segment.primitives) {
      item["primitives"].push_back(YAML::Node{});
      item["primitives"][item["primitives"].size() - 1]["id"] = primitive.id;
      item["primitives"][item["primitives"].size() - 1]["primitive_type"] = primitive.primitive_type;
    }
    node["segments"].push_back(item);
  }
  return node;
}

template <typename Message>
YAML::Node message_yaml(const Message &);

template <> YAML::Node message_yaml(const nav_msgs::Odometry & value) { YAML::Node n; n["header"] = header_yaml(value.header); n["child_frame_id"] = value.child_frame_id; n["pose"] = pose_covariance_yaml(value.pose); n["twist"] = twist_covariance_yaml(value.twist); return n; }
template <> YAML::Node message_yaml(const geometry_msgs::AccelWithCovarianceStamped & value) { YAML::Node n; n["header"] = header_yaml(value.header); n["accel"] = accel_covariance_yaml(value.accel); return n; }
template <> YAML::Node message_yaml(const autoware_perception_msgs::PredictedObjects & value) { YAML::Node n; n["header"] = header_yaml(value.header); for (const auto & object : value.objects) n["objects"].push_back(predicted_object_yaml(object)); return n; }
template <> YAML::Node message_yaml(const autoware_adapi_v1_msgs::OperationModeState & value) { YAML::Node n; n["stamp"] = time_yaml(value.stamp); n["mode"] = value.mode; n["is_autoware_control_enabled"] = value.is_autoware_control_enabled; n["is_in_transition"] = value.is_in_transition; n["is_stop_mode_available"] = value.is_stop_mode_available; n["is_autonomous_mode_available"] = value.is_autonomous_mode_available; n["is_local_mode_available"] = value.is_local_mode_available; n["is_remote_mode_available"] = value.is_remote_mode_available; return n; }
template <> YAML::Node message_yaml(const autoware_planning_msgs::LaneletRoute & value) { return route_yaml(value); }
template <> YAML::Node message_yaml(const autoware_perception_msgs::TrafficLightGroupArray & value) { YAML::Node n; n["stamp"] = time_yaml(value.stamp); for (const auto & group : value.traffic_light_groups) { YAML::Node item; item["traffic_light_group_id"] = group.traffic_light_group_id; for (const auto & element : group.elements) { YAML::Node e; e["color"] = element.color; e["shape"] = element.shape; e["status"] = element.status; e["confidence"] = element.confidence; item["elements"].push_back(e); } n["traffic_light_groups"].push_back(item); } return n; }
template <> YAML::Node message_yaml(const autoware_perception_msgs::TrackedObjects & value) { YAML::Node n; n["header"] = header_yaml(value.header); for (const auto & object : value.objects) n["objects"].push_back(tracked_object_yaml(object)); return n; }
template <> YAML::Node message_yaml(const autoware_internal_planning_msgs::PathWithLaneId & value) { YAML::Node n; n["header"] = header_yaml(value.header); for (const auto & point : value.points) n["points"].push_back(path_point_yaml(point)); for (const auto & point : value.left_bound) n["left_bound"].push_back(point_yaml(point)); for (const auto & point : value.right_bound) n["right_bound"].push_back(point_yaml(point)); return n; }

class TopicSnapShotSaver
{
public:
  TopicSnapShotSaver(
    std::string map_path, std::string map_path_uri, const std::string & config_path,
    ros::NodeHandle & node)
  : map_path_(std::move(map_path)), map_path_uri_(std::move(map_path_uri)), node_(node)
  {
    server_ = node_.advertiseService(
      "/autoware_test_utils/topic_snapshot_saver", &TopicSnapShotSaver::on_service, this);
    const auto config = YAML::LoadFile(config_path);
    for (const auto & field : config["fields"]) {
      const auto index = get_topic_index(field["type"].as<std::string>());
      if (!index) {
        ROS_WARN_STREAM("Ignoring unsupported snapshot type '" << field["type"].as<std::string>() << "'");
        continue;
      }
      const auto name = field["name"].as<std::string>();
      const auto topic = field["topic"].as<std::string>();
      field_2_topic_type_[name] = *index;
      switch (*index) {
        case 0: add_subscription<0>(topic); break;
        case 1: add_subscription<1>(topic); break;
        case 2: add_subscription<2>(topic); break;
        case 3: add_subscription<3>(topic); break;
        case 4: add_subscription<4>(topic); break;
        case 5: add_subscription<5>(topic); break;
        case 6: add_subscription<6>(topic); break;
        case 7: add_subscription<7>(topic); break;
      }
    }
  }

private:
  template <size_t I>
  void add_subscription(const std::string & topic)
  {
    using Msg = RosMsgType<I>;
    auto buffer = std::make_shared<MessageType>(Msg{});
    message_buffer_[I] = buffer;
    subscribers_[I] = node_.subscribe<Msg>(
      topic, 1, [buffer](const typename Msg::ConstPtr & msg) {
        std::lock_guard<std::mutex> lock(g_mutex);
        *buffer = *msg;
      });
  }

  template <size_t I>
  void write_message(YAML::Node & yaml, const std::string & field)
  {
    const auto it = message_buffer_.find(I);
    if (it != message_buffer_.end()) {
      yaml[field] = message_yaml(std::get<RosMsgType<I>>(*it->second));
    }
  }

  bool on_service(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    YAML::Node yaml;
    yaml["format_version"] = 1;
    yaml["map_path_uri"] = map_path_uri_;
    for (const auto & entry : field_2_topic_type_) {
      switch (entry.second) {
        case 0: write_message<0>(yaml, entry.first); break;
        case 1: write_message<1>(yaml, entry.first); break;
        case 2: write_message<2>(yaml, entry.first); break;
        case 3: write_message<3>(yaml, entry.first); break;
        case 4: write_message<4>(yaml, entry.first); break;
        case 5: write_message<5>(yaml, entry.first); break;
        case 6: write_message<6>(yaml, entry.first); break;
        case 7: write_message<7>(yaml, entry.first); break;
      }
    }
    std::ofstream ofs("topic_snapshot.yaml");
    if (!ofs) {
      ROS_ERROR("Failed to open topic_snapshot.yaml");
      return false;
    }
    ofs << "# AUTO GENERATED by autoware_test_utils::topic_snapshot_saver\n";
    ofs << "# format_version: 1\n";
    ofs << yaml;
    ROS_INFO("Saved planner data to topic_snapshot.yaml");
    return true;
  }

  const std::string map_path_;
  const std::string map_path_uri_;
  ros::NodeHandle & node_;
  ros::ServiceServer server_;
  std::unordered_map<size_t, ros::Subscriber> subscribers_;
  std::unordered_map<size_t, std::shared_ptr<MessageType>> message_buffer_;
  std::unordered_map<std::string, size_t> field_2_topic_type_;
};
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "topic_snapshot_saver_frontend");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  std::string map_path_uri{"none"};
  std::string config_path_uri{"none"};
  private_node.param("map_path", map_path_uri, map_path_uri);
  private_node.param("config_path", config_path_uri, config_path_uri);
  if (map_path_uri == "none" || config_path_uri == "none") {
    ROS_ERROR("~map_path and ~config_path are required");
    return 2;
  }
  const auto map_path = autoware::test_utils::resolve_pkg_share_uri(map_path_uri);
  const auto config_path = autoware::test_utils::resolve_pkg_share_uri(config_path_uri);
  if (!map_path || !config_path) {
    ROS_ERROR("Could not resolve package:// URI passed to ~map_path or ~config_path");
    return 2;
  }
  TopicSnapShotSaver saver(*map_path, map_path_uri, *config_path, node);
  ros::spin();
  return 0;
}
