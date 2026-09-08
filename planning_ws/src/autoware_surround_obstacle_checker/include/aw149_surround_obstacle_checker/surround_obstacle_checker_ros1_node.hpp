#pragma once

#include <aw149_surround_obstacle_checker/parameters.hpp>
#include <autoware_internal_debug_msgs/Float64Stamped.h>
#include <autoware_internal_planning_msgs/VelocityLimit.h>
#include <autoware_internal_planning_msgs/VelocityLimitClearCommand.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <unique_identifier_msgs/UUID.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <mutex>
#include <optional>
#include <string>

namespace aw149::surround_obstacle_checker
{
enum class State { PASS, STOP };

// The ROS2 source carries this value from obstacle selection through the
// debug/planning-factor publisher.  Keep the payload intact instead of
// reducing a detected obstacle to a boolean stop decision.
struct ObstacleObservation
{
  double distance{0.0};
  bool is_pointcloud{false};
  geometry_msgs::Point map_point;
  unique_identifier_msgs::UUID object_id;
};

class SurroundObstacleCheckerRos1Node
{
public:
  SurroundObstacleCheckerRos1Node(ros::NodeHandle node, ros::NodeHandle private_node);

private:
  void onOdometry(const nav_msgs::Odometry::ConstPtr & message);
  void onPointcloud(const sensor_msgs::PointCloud2::ConstPtr & message);
  void onObjects(const autoware_perception_msgs::PredictedObjects::ConstPtr & message);
  void onTimer(const ros::TimerEvent &);
  std::optional<ObstacleObservation> nearestPointcloud(
    const sensor_msgs::PointCloud2::ConstPtr & pointcloud,
    const nav_msgs::Odometry::ConstPtr & odometry) const;
  std::optional<ObstacleObservation> nearestObject(
    const autoware_perception_msgs::PredictedObjects::ConstPtr & objects,
    const nav_msgs::Odometry::ConstPtr & odometry) const;
  double distanceToEgoRectangle(double x, double y, const ObstacleTypeParameters & type) const;
  bool stopRequired(bool obstacle_found, bool vehicle_stopped, double threshold);
  void publishDebug(
    const nav_msgs::Odometry::ConstPtr & odometry,
    const std::optional<ObstacleObservation> & nearest) const;
  geometry_msgs::PolygonStamped footprint(double front_margin, double side_margin, double rear_margin) const;
  static std::optional<std::string> labelName(uint8_t label);

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  Parameters parameters_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  mutable tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber objects_sub_;
  ros::Publisher velocity_limit_pub_;
  ros::Publisher clear_command_pub_;
  ros::Publisher processing_time_pub_;
  ros::Publisher debug_marker_pub_;
  ros::Publisher footprint_pub_;
  ros::Publisher footprint_offset_pub_;
  ros::Publisher footprint_recover_offset_pub_;
  ros::Timer timer_;
  std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface> planning_factor_interface_;
  mutable std::mutex input_mutex_;
  nav_msgs::Odometry::ConstPtr odometry_;
  sensor_msgs::PointCloud2::ConstPtr pointcloud_;
  autoware_perception_msgs::PredictedObjects::ConstPtr objects_;
  State state_{State::PASS};
  std::optional<ros::Time> last_obstacle_found_time_;
};
}  // namespace aw149::surround_obstacle_checker
