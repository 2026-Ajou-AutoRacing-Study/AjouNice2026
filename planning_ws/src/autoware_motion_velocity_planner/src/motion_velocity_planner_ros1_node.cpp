#include <autoware/motion_velocity_planner/motion_velocity_planner_ros1_node.hpp>

#include <autoware/route_handler/route_handler.hpp>

#include <xmlrpcpp/XmlRpcValue.h>

#include <pcl_conversions/pcl_conversions.h>
#include <autoware/velocity_smoother/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.hpp>

namespace autoware::motion_velocity_planner
{
MotionVelocityPlannerRos1Node::MotionVelocityPlannerRos1Node(ros::NodeHandle private_node)
: private_node_(std::move(private_node))
{
  XmlRpc::XmlRpcValue launch_modules;
  has_launch_module_configuration_ = private_node_.getParam("launch_modules", launch_modules);
  if (has_launch_module_configuration_) {
    if (launch_modules.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_ERROR_STREAM_NAMED(private_node_.getNamespace(), "launch_modules must be a ROS1 string array.");
    } else {
      for (int index = 0; index < launch_modules.size(); ++index) {
        if (launch_modules[index].getType() != XmlRpc::XmlRpcValue::TypeString) {
          ROS_ERROR_STREAM_NAMED(private_node_.getNamespace(), "launch_modules contains a non-string entry.");
          continue;
        }
        const auto module_name = static_cast<std::string>(launch_modules[index]);
        if (manager_.launchModule(private_node_, module_name) &&
            module_name == "autoware::motion_velocity_planner::DynamicObstacleStopModule") {
          dynamic_obstacle_stop_loaded_ = true;
        }
      }
    }
  }
  trajectory_subscriber_ = private_node_.subscribe(
    "input/trajectory", 1, &MotionVelocityPlannerRos1Node::onTrajectory, this);
  if (manager_.requiredSubscriptions().predicted_objects) {
    predicted_objects_subscriber_ = private_node_.subscribe(
      "input/predicted_objects", 1, &MotionVelocityPlannerRos1Node::onPredictedObjects, this);
  }
  if (manager_.requiredSubscriptions().no_ground_pointcloud) {
    no_ground_pointcloud_subscriber_ = private_node_.subscribe(
      "input/no_ground_pointcloud", 1, &MotionVelocityPlannerRos1Node::onNoGroundPointcloud, this);
  }
  if (manager_.requiredSubscriptions().occupancy_grid_map) {
    occupancy_grid_map_subscriber_ = private_node_.subscribe(
      "input/occupancy_grid_map", 1, &MotionVelocityPlannerRos1Node::onOccupancyGridMap, this);
  }
  if (manager_.requiredSubscriptions().lanelet_map) {
    lanelet_map_subscriber_ = private_node_.subscribe(
      "input/lanelet_map", 1, &MotionVelocityPlannerRos1Node::onLaneletMap, this);
  }
  odometry_subscriber_ = private_node_.subscribe("input/odometry", 1, &MotionVelocityPlannerRos1Node::onOdometry, this);
  acceleration_subscriber_ = private_node_.subscribe("input/acceleration", 1, &MotionVelocityPlannerRos1Node::onAcceleration, this);
  private_node_.param("vehicle_info/max_longitudinal_offset_m",
    planner_data_->vehicle_info_.max_longitudinal_offset_m, 0.0);
  private_node_.param("vehicle_info/rear_overhang_m",
    planner_data_->vehicle_info_.rear_overhang_m, 0.0);
  private_node_.param("vehicle_info/vehicle_width_m", planner_data_->vehicle_info_.vehicle_width_m, 0.0);
  if (dynamic_obstacle_stop_loaded_) {
    try {
      rclcpp::Node smoother_parameters(private_node_);
      planner_data_->velocity_smoother_ = std::make_shared<
        autoware::velocity_smoother::AnalyticalJerkConstrainedSmoother>(smoother_parameters);
    } catch (const std::exception & error) {
      velocity_smoother_ready_ = false;
      ROS_ERROR_STREAM_NAMED(
        private_node_.getNamespace(),
        "DynamicObstacleStop remains not ready: configured velocity-smoother is required: " << error.what());
    }
  }
  trajectory_publisher_ = private_node_.advertise<autoware_planning_msgs::Trajectory>("output/trajectory", 1);
  velocity_limit_publisher_ = private_node_.advertise<autoware_internal_planning_msgs::VelocityLimit>(
    "output/velocity_limit", 1, true);
  clear_velocity_limit_publisher_ =
    private_node_.advertise<autoware_internal_planning_msgs::VelocityLimitClearCommand>(
      "output/clear_velocity_limit", 1, true);
}

void MotionVelocityPlannerRos1Node::onTrajectory(const autoware_planning_msgs::TrajectoryConstPtr & message)
{
  if (manager_.requiredSubscriptions().predicted_objects && !predicted_objects_received_) {
    ROS_WARN_THROTTLE(1.0, "motion_velocity_planner waits for required predicted_objects input");
    return;
  }
  if (manager_.requiredSubscriptions().no_ground_pointcloud && !no_ground_pointcloud_received_) {
    ROS_WARN_THROTTLE(1.0, "motion_velocity_planner waits for required no_ground_pointcloud input");
    return;
  }
  if (manager_.requiredSubscriptions().occupancy_grid_map && !occupancy_grid_map_received_) {
    ROS_WARN_THROTTLE(1.0, "motion_velocity_planner waits for required occupancy_grid_map input");
    return;
  }
  if (manager_.requiredSubscriptions().lanelet_map && !lanelet_map_received_) {
    ROS_WARN_THROTTLE(1.0, "motion_velocity_planner waits for required lanelet_map input");
    return;
  }
  if (!odometry_received_ || !acceleration_received_) {
    ROS_WARN_THROTTLE(1.0, "motion_velocity_planner waits for required odometry and acceleration input");
    return;
  }
  if (dynamic_obstacle_stop_loaded_ && !velocity_smoother_ready_) {
    ROS_ERROR_THROTTLE(1.0, "motion_velocity_planner waits for configured velocity_smoother for DynamicObstacleStop");
    return;
  }
  // Modules such as RunOut consume the smoothed trajectory independently of
  // DynamicObstacleStop.  In the ROS1 host, DynamicObstacleStop is the only
  // optional module that actively replaces this sequence, so retain the raw
  // input as the safe fallback when that module is not configured.
  auto smoothed_trajectory_points = message->points;
  if (dynamic_obstacle_stop_loaded_) {
    std::vector<std::vector<autoware_planning_msgs::TrajectoryPoint>> debug_trajectories;
    const auto initial_velocity = planner_data_->current_odometry.twist.twist.linear.x;
    const auto initial_acceleration = planner_data_->current_acceleration.accel.accel.linear.x;
    if (!planner_data_->velocity_smoother_->apply(
          initial_velocity, initial_acceleration, message->points, smoothed_trajectory_points,
          debug_trajectories, false)) {
      ROS_ERROR_THROTTLE(
        1.0, "motion_velocity_planner refuses DynamicObstacleStop planning after velocity smoothing failure");
      return;
    }
  }
  const auto planning_results =
    manager_.planVelocities(message->points, smoothed_trajectory_points, planner_data_);
  auto output = *message;
  output.points = core_.applyResults(message->points, planning_results);
  // Source-equivalent result transport: some modules (notably ObstacleCruise)
  // constrain the downstream smoother through VelocityLimit rather than by
  // modifying this trajectory directly.
  for (const auto & planning_result : planning_results) {
    if (planning_result.velocity_limit) {
      velocity_limit_publisher_.publish(*planning_result.velocity_limit);
    }
    if (planning_result.velocity_limit_clear_command) {
      clear_velocity_limit_publisher_.publish(*planning_result.velocity_limit_clear_command);
    }
  }
  trajectory_publisher_.publish(output);
}

void MotionVelocityPlannerRos1Node::onPredictedObjects(
  const autoware_perception_msgs::PredictedObjectsConstPtr & message)
{
  planner_data_->process_predicted_objects(*message);
  predicted_objects_received_ = true;
}

void MotionVelocityPlannerRos1Node::onNoGroundPointcloud(
  const sensor_msgs::PointCloud2ConstPtr & message)
{
  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  pcl::fromROSMsg(*message, pointcloud);
  planner_data_->no_ground_pointcloud.preprocess_pointcloud(std::move(pointcloud));
  no_ground_pointcloud_received_ = true;
}

void MotionVelocityPlannerRos1Node::onOccupancyGridMap(const nav_msgs::OccupancyGridConstPtr & message)
{
  planner_data_->occupancy_grid_map = *message;
  occupancy_grid_map_received_ = true;
}

void MotionVelocityPlannerRos1Node::onLaneletMap(const autoware_map_msgs::LaneletMapBinConstPtr & message)
{
  planner_data_->route_handler = std::make_shared<autoware::route_handler::RouteHandler>(*message);
  lanelet_map_received_ = planner_data_->route_handler->isMapMsgReady();
}

void MotionVelocityPlannerRos1Node::onOdometry(const nav_msgs::OdometryConstPtr & message)
{
  planner_data_->current_odometry = *message;
  odometry_received_ = true;
}

void MotionVelocityPlannerRos1Node::onAcceleration(const geometry_msgs::AccelWithCovarianceStampedConstPtr & message)
{
  planner_data_->current_acceleration = *message;
  acceleration_received_ = true;
}
}  // namespace autoware::motion_velocity_planner
