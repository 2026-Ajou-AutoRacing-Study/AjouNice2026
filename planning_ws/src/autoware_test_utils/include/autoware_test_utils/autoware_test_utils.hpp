// Copyright 2023 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_TEST_UTILS__AUTOWARE_TEST_UTILS_HPP_
#define AUTOWARE_TEST_UTILS__AUTOWARE_TEST_UTILS_HPP_

#include <ros/package.h>
#include <ros/ros.h>

#include <autoware_adapi_v1_msgs/OperationModeState.h>
#include <autoware_internal_planning_msgs/PathPointWithLaneId.h>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_internal_planning_msgs/Scenario.h>
#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_planning_msgs/LaneletPrimitive.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_planning_msgs/LaneletSegment.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <tf2_msgs/TFMessage.h>

#include <lanelet2_io/Io.h>

#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::test_utils
{
using autoware_adapi_v1_msgs::OperationModeState;
using autoware_internal_planning_msgs::PathPointWithLaneId;
using autoware_internal_planning_msgs::PathWithLaneId;
using autoware_map_msgs::LaneletMapBin;
using autoware_planning_msgs::LaneletPrimitive;
using autoware_planning_msgs::LaneletRoute;
using autoware_planning_msgs::LaneletSegment;
using autoware_planning_msgs::Path;
using autoware_planning_msgs::Trajectory;
using RouteSections = std::vector<autoware_planning_msgs::LaneletSegment>;
using autoware_internal_planning_msgs::Scenario;
using geometry_msgs::Point;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using nav_msgs::OccupancyGrid;
using nav_msgs::Odometry;
using tf2_msgs::TFMessage;

/**
 * @brief Creates a Pose message with the specified position and orientation.
 *
 * This function initializes a geometry_msgs::Pose message with the
 * given position (x, y, z) and orientation (roll, pitch, yaw).
 *
 * @param x The x-coordinate of the position.
 * @param y The y-coordinate of the position.
 * @param z The z-coordinate of the position.
 * @param roll The roll component of the orientation (in radians).
 * @param pitch The pitch component of the orientation (in radians).
 * @param yaw The yaw component of the orientation (in radians).
 * @return A geometry_msgs::Pose message with the specified position
 * and orientation.
 */
geometry_msgs::Pose createPose(
  double x, double y, double z, double roll, double pitch, double yaw);

/**
 * @brief Creates a Pose message from a 4-element array representing a 3D pose.
 *
 * This function initializes a geometry_msgs::Pose message using a
 * std::array of four doubles, where the first three elements represent the
 * position (x, y, z) and the fourth element represents the yaw orientation.
 * The roll and pitch orientations are assumed to be zero.
 *
 * @param pose3d A std::array of four doubles representing the 3D pose. The
 * first element is the x-coordinate, the second is the y-coordinate, the
 * third is the z-coordinate, and the fourth is the yaw orientation.
 * @return A geometry_msgs::Pose message with the specified position
 * and yaw orientation, with roll and pitch set to zero.
 */
geometry_msgs::Pose createPose(const std::array<double, 4> & pose3d);

/**
 * @brief Creates a LaneletSegment with the specified ID.
 *
 * Initializes a LaneletSegment containing a single LaneletPrimitive with the
 * given ID and a primitive type of "lane".
 *
 * @param id The ID for the LaneletPrimitive and preferred primitive.
 * @return A LaneletSegment with the specified ID.
 */
LaneletSegment createLaneletSegment(int id);

/**
 * @brief Loads a Lanelet map from a specified file.
 *
 * This function loads a Lanelet2 map using the given filename. It uses the MGRS
 * projector and checks for any errors during the loading process. If errors
 * are found, they are logged, and the function returns nullptr.
 *
 * @param lanelet2_filename The filename of the Lanelet2 map to load.
 * @return A LaneletMapPtr to the loaded map, or nullptr if there were errors.
 */
lanelet::LaneletMapPtr loadMap(const std::string & lanelet2_filename);

/**
 * @brief Converts a Lanelet map to a LaneletMapBin message.
 *
 * This function converts a given Lanelet map to a LaneletMapBin message. It also
 * parses the format and map versions from the specified filename and includes
 * them in the message. The timestamp for the message is set to the provided time.
 *
 * @param map The Lanelet map to convert.
 * @param lanelet2_filename The filename of the Lanelet2 map, used to parse format and map versions.
 * @param now The current time to set in the message header.
 * @return A LaneletMapBin message containing the converted map data.
 */
LaneletMapBin convertToMapBinMsg(
  const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename,
  const ros::Time & now);

/**
 * @brief Creates a normal Lanelet route with predefined start and goal poses.
 *
 * This function initializes a LaneletRoute with a predefined start and goal pose.
 *
 * @return A LaneletRoute with the specified start and goal poses.
 */
LaneletRoute makeNormalRoute();

/**
 * @brief Creates an OccupancyGrid message with the specified dimensions and resolution.
 *
 * This function initializes an OccupancyGrid message with given width, height, and resolution.
 * All cells in the grid are initialized to zero.
 *
 * @param width The width of the occupancy grid.
 * @param height The height of the occupancy grid.
 * @param resolution The resolution of the occupancy grid.
 * @return An OccupancyGrid message with the specified dimensions and resolution.
 */
OccupancyGrid makeCostMapMsg(size_t width = 150, size_t height = 150, double resolution = 0.2);

/**
 * @brief Get the absolute path to the lanelet map file.
 *
 * This function retrieves the absolute path to a lanelet map file located in the
 * package's share directory under the "test_map" folder.
 *
 * @param package_name The name of the package containing the map file.
 * @param map_filename The name of the map file.
 * @return A string representing the absolute path to the lanelet map file.
 */
std::string get_absolute_path_to_lanelet_map(
  const std::string & package_name, const std::string & map_filename);

/**
 * @brief Get the absolute path to the route file.
 *
 * This function retrieves the absolute path to a route file located in the
 * package's share directory under the "test_route" folder.
 *
 * @param package_name The name of the package containing the route file.
 * @param route_filename The name of the route file.
 * @return A string representing the absolute path to the route file.
 */
std::string get_absolute_path_to_route(
  const std::string & package_name, const std::string & route_filename);

/**
 * @brief Get the absolute path to the config file.
 *
 * This function retrieves the absolute path to a route file located in the
 * package's share directory under the "config" folder.
 *
 * @param package_name The name of the package containing the route file.
 * @param route_filename The name of the config file.
 * @return A string representing the absolute path to the config file.
 */
std::string get_absolute_path_to_config(
  const std::string & package_name, const std::string & config_filename);

/**
 * @brief Creates a LaneletMapBin message from a Lanelet map file.
 *
 * This function loads a Lanelet map from the given file, overwrites the
 * centerline with the specified resolution, and converts the map to a LaneletMapBin message.
 *
 * @param absolute_path The absolute path to the Lanelet2 map file.
 * @param center_line_resolution The resolution for the centerline.
 * @return A LaneletMapBin message containing the map data.
 */
LaneletMapBin make_map_bin_msg(
  const std::string & absolute_path, const double center_line_resolution = 5.0);

/**
 * @brief Creates a LaneletMapBin message using a Lanelet2 map file.
 *
 * This function loads a specified map file from the test_map folder in the
 * specified package (or autoware_test_utils if no package is specified),
 * overwrites the centerline with a resolution of 5.0,
 * and converts the map to a LaneletMapBin message.
 *
 * @param package_name The name of the package containing the map file. If empty, defaults to
 * "autoware_test_utils".
 * @param map_filename The name of the map file (e.g. "lanelet2_map.osm")
 * @return A LaneletMapBin message containing the map data.
 */
LaneletMapBin makeMapBinMsg(
  const std::string & package_name = "autoware_test_utils",
  const std::string & map_filename = "lanelet2_map.osm");

/**
 * @brief Creates an Odometry message with a specified shift.
 *
 * This function initializes an Odometry message with a pose shifted by the given amount at y
 * direction. x pose, z pose, and yaw angle remains zero.
 *
 * @param shift The amount by which to shift the pose.
 * @return An Odometry message with the specified shift.
 */
Odometry makeOdometry(const double shift = 0.0);

/**
 * @brief Creates an initial Odometry message with a specified shift.
 *
 * This function initializes an Odometry message with a pose shifted by the given amount,
 * accounting for a specific yaw angle.
 *
 * @param shift The amount by which to shift the pose.
 * @return An Odometry message with the specified shift.
 */
Odometry makeInitialPose(const double shift = 0.0);

/**
 * @brief Creates a TFMessage with the specified frame IDs.
 *
 * This function initializes a TFMessage with a transform containing the given frame IDs.
 * The transform includes a specific translation and rotation.
 *
 * @param target_node The node to use for obtaining the current time.
 * @param frame_id The ID of the parent frame.
 * @param child_frame_id The ID of the child frame.
 * @return A TFMessage containing the transform.
 */
TFMessage makeTFMsg(
  const ros::Time & now = ros::Time::now(), std::string && frame_id = "",
  std::string && child_frame_id = "");

/**
 * @brief Creates a Scenario message with the specified scenario.
 *
 * This function initializes a Scenario message with the current scenario and a list of activating
 * scenarios.
 *
 * @param scenario The name of the current scenario.
 * @return A Scenario message with the specified scenario.
 */
Scenario makeScenarioMsg(const std::string & scenario);

/**
 * @brief Combines two sets of consecutive RouteSections.
 *
 * This function combines two sets of RouteSections, removing the overlapping end section from the
 * first set.
 *
 * @param route_sections1 The first set of RouteSections.
 * @param route_sections2 The second set of RouteSections.
 * @return A combined set of RouteSections.
 */
RouteSections combineConsecutiveRouteSections(
  const RouteSections & route_sections1, const RouteSections & route_sections2);

/**
 * @brief Creates a predefined behavior Lanelet route.
 *
 * This function initializes a LaneletRoute with predefined start and goal poses,
 * a list of lanelet segment IDs, and a fixed UUID.
 * this is for the test lanelet2_map.osm
 * file hash: a9f84cff03b55a64917bc066451276d2293b0a54f5c088febca0c7fdf2f245d5
 *
 * @return A LaneletRoute with the specified attributes.
 */
LaneletRoute makeBehaviorNormalRoute();

/**
 * @brief Creates a predefined behavior Lanelet route with the goal on the left side.
 *
 * This function initializes a LaneletRoute with predefined start and goal poses,
 * a list of lanelet segment IDs, and a fixed UUID.
 * this is for the test lanelet2_map.osm
 * file hash: a9f84cff03b55a64917bc066451276d2293b0a54f5c088febca0c7fdf2f245d5
 *
 * @return A LaneletRoute with the specified attributes.
 */
LaneletRoute makeBehaviorGoalOnLeftSideRoute();

/**
 * @brief Spins multiple ROS nodes a specified number of times.
 *
 * This function spins the given test and target nodes for the specified number of iterations.
 *
 * @param test_node The test node to spin.
 * @param target_node The target node to spin.
 * @param repeat_count The number of times to spin the nodes.
 */
void spinSomeNodes(const int repeat_count = 1);

/**
 * @brief Updates node options with parameter files.
 *
 * This function updates the given node options to include the specified parameter files.
 *
 * @param node_options The node options to update.
 * @param params_files A vector of parameter file paths to add to the node options.
 */
/**
 * @brief Loads ROS1 private parameters from the supplied YAML files.
 *
 * ROS1 has no NodeOptions argument vector.  The files are loaded into the
 * current node's private namespace, which is the equivalent test setup
 * contract before constructing the node under test.
 */
void loadPrivateParameters(const std::vector<std::string> & params_files);

/**
 * @brief Loads a PathWithLaneId message from a YAML file.
 *
 * This function loads a PathWithLaneId message from a YAML file located in the
 * autoware_test_utils package.
 *
 * @return A PathWithLaneId message containing the loaded data.
 */
PathWithLaneId loadPathWithLaneIdInYaml();

/**
 * @brief create a straight lanelet from 2 segments defined by 4 points
 * @param [in] left0 start of the left segment
 * @param [in] left1 end of the left segment
 * @param [in] right0 start of the right segment
 * @param [in] right1 end of the right segment
 * @return a ConstLanelet with the given left and right bounds and a unique lanelet id
 *
 */
lanelet::ConstLanelet make_lanelet(
  const lanelet::BasicPoint2d & left0, const lanelet::BasicPoint2d & left1,
  const lanelet::BasicPoint2d & right0, const lanelet::BasicPoint2d & right1);

/**
 * @brief Generates a trajectory with specified parameters.
 *
 * This function generates a trajectory of type T with a given number of points,
 * point interval, velocity, initial theta, delta theta, and optionally an
 * overlapping point index.
 *
 * @tparam T The type of the trajectory.
 * @param num_points The number of points in the trajectory.
 * @param point_interval The distance between consecutive points.
 * @param velocity The longitudinal velocity for each point.
 * @param init_theta The initial theta angle.
 * @param delta_theta The change in theta per point.
 * @param overlapping_point_index The index of the point to overlap (default is max size_t value).
 * @return A trajectory of type T with the specified parameters.
 */
template <class T>
T generateTrajectory(
  const size_t num_points, const double point_interval, const double velocity = 0.0,
  const double init_theta = 0.0, const double delta_theta = 0.0,
  const size_t overlapping_point_index = std::numeric_limits<size_t>::max())
{
  using Point = typename T::_points_type::value_type;

  T traj;
  traj.header.stamp = ros::Time::now();
  for (size_t i = 0; i < num_points; ++i) {
    const double theta = init_theta + static_cast<double>(i) * delta_theta;
    const double x = static_cast<double>(i) * point_interval * std::cos(theta);
    const double y = static_cast<double>(i) * point_interval * std::sin(theta);

    Point p;
    p.pose = createPose(x, y, 0.0, 0.0, 0.0, theta);
    p.longitudinal_velocity_mps = velocity;
    traj.points.push_back(p);

    if (i == overlapping_point_index) {
      Point value_to_insert = traj.points[overlapping_point_index];
      traj.points.insert(traj.points.begin() + overlapping_point_index + 1, value_to_insert);
    }
  }

  return traj;
}

template <>
inline PathWithLaneId generateTrajectory<PathWithLaneId>(
  const size_t num_points, const double point_interval, const double velocity,
  const double init_theta, const double delta_theta, const size_t overlapping_point_index)
{
  PathWithLaneId traj;
  traj.header.stamp = ros::Time::now();

  for (size_t i = 0; i < num_points; i++) {
    const double theta = init_theta + static_cast<double>(i) * delta_theta;
    const double x = static_cast<double>(i) * point_interval * std::cos(theta);
    const double y = static_cast<double>(i) * point_interval * std::sin(theta);

    PathPointWithLaneId p;
    p.point.pose = createPose(x, y, 0.0, 0.0, 0.0, theta);
    p.point.longitudinal_velocity_mps = static_cast<float>(velocity);
    p.lane_ids.push_back(static_cast<int64_t>(i));
    traj.points.push_back(p);

    if (i == overlapping_point_index) {
      auto value_to_insert = traj.points.at(overlapping_point_index);
      traj.points.insert(
        traj.points.begin() + static_cast<int64_t>(overlapping_point_index) + 1, value_to_insert);
    }
  }
  return traj;
}

/**
 * @brief Creates a publisher with appropriate QoS settings.
 *
 * This function creates a publisher for a given topic name and message type with appropriate
 * QoS settings, depending on the message type.
 *
 * @tparam T The type of the message to publish.
 * @param test_node The node to create the publisher on.
 * @param topic_name The name of the topic to publish to.
 * @param publisher A reference to the publisher to be created.
 */
template <typename T>
void createPublisherWithQoS(
  ros::NodeHandle & test_node, const std::string & topic_name, ros::Publisher & publisher)
{
  // ROS1 latch is the direct analogue of the ROS2 transient-local contracts.
  if constexpr (
    std::is_same_v<T, LaneletRoute> || std::is_same_v<T, LaneletMapBin> ||
    std::is_same_v<T, OperationModeState>) {
    publisher = test_node.advertise<T>(topic_name, 1, true);
  } else {
    publisher = test_node.advertise<T>(topic_name, 1, false);
  }
}

/**
 * @brief Sets up a publisher for a given topic.
 *
 * This function sets up a publisher for a given topic using createPublisherWithQoS.
 *
 * @tparam T The type of the message to publish.
 * @param test_node The node to create the publisher on.
 * @param topic_name The name of the topic to publish to.
 * @param publisher A reference to the publisher to be set.
 */
template <typename T>
void setPublisher(
  ros::NodeHandle & test_node, const std::string & topic_name, ros::Publisher & publisher)
{
  createPublisherWithQoS<T>(test_node, topic_name, publisher);
}

/**
 * @brief Creates a subscription with appropriate QoS settings.
 *
 * This function creates a subscription for a given topic name and message type with appropriate
 * QoS settings, depending on the message type.
 *
 * @tparam T The type of the message to subscribe to.
 * @param test_node The node to create the subscription on.
 * @param topic_name The name of the topic to subscribe to.
 * @param callback The callback function to call when a message is received.
 * @param subscriber A reference to the subscription to be created.
 * @param qos The QoS settings for the subscription (optional).
 */
template <typename T>
void createSubscription(
  ros::NodeHandle & test_node, const std::string & topic_name,
  const std::function<void(const typename T::ConstPtr)> & callback, ros::Subscriber & subscriber,
  const uint32_t queue_size = 10)
{
  subscriber = test_node.subscribe<T>(topic_name, queue_size, callback);
}

/**
 * @brief Sets up a subscriber for a given topic.
 *
 * This function sets up a subscriber for a given topic using createSubscription.
 *
 * @tparam T The type of the message to subscribe to.
 * @param test_node The node to create the subscription on.
 * @param topic_name The name of the topic to subscribe to.
 * @param subscriber A reference to the subscription to be set.
 * @param count A reference to a counter that increments on message receipt.
 * @param qos The QoS settings for the subscription (optional).
 */
template <typename T>
void setSubscriber(
  ros::NodeHandle & test_node, const std::string & topic_name, ros::Subscriber & subscriber,
  size_t & count, const uint32_t queue_size = 10)
{
  createSubscription(
    test_node, topic_name, [&count](const typename T::ConstPtr) { count++; }, subscriber,
    queue_size);
}

/**
 * @brief Publishes data to a target node.
 *
 * This function publishes data to a target node on a given topic, ensuring that the topic has
 * subscribers and retrying a specified number of times.
 *
 * @tparam T The type of the message to publish.
 * @param test_node The node to create the publisher on.
 * @param target_node The target node to publish the message to.
 * @param topic_name The name of the topic to publish to.
 * @param publisher A shared pointer to the publisher.
 * @param data The data to publish.
 * @param repeat_count The number of times to retry publishing.
 */
template <typename T>
void publishToTargetNode(
  ros::NodeHandle & test_node, ros::NodeHandle &, const std::string & topic_name,
  ros::Publisher & publisher, const T & data, const int repeat_count = 3)
{
  if (topic_name.empty()) {
    int status = 1;
    char * demangled_name = abi::__cxa_demangle(typeid(data).name(), nullptr, nullptr, &status);
    if (status == 0) {
      throw std::runtime_error(std::string("Topic name for ") + demangled_name + " is empty");
    }
    throw std::runtime_error(std::string("Topic name for ") + typeid(data).name() + " is empty");
  }

  autoware::test_utils::setPublisher<T>(test_node, topic_name, publisher);
  publisher.publish(data);

  if (publisher.getNumSubscribers() == 0) {
    throw std::runtime_error("No subscriber for " + topic_name);
  }
  autoware::test_utils::spinSomeNodes(repeat_count);
}

/**
 * @brief Manages publishing and subscribing to ROS topics for testing Autoware.
 *
 * The AutowareTestManager class provides utility functions to facilitate
 * the publishing of messages to specified topics and the setting up of
 * subscribers to listen for messages on specified topics. This class
 * simplifies the setup of test environments in Autoware.
 */
class AutowareTestManager
{
public:
  AutowareTestManager() : test_node_(), pub_clock_(test_node_.advertise<rosgraph_msgs::Clock>("/clock", 1)) {}

  template <typename MessageType>
  void test_pub_msg(
    ros::NodeHandle & target_node, const std::string & topic_name, const MessageType & msg)
  {
    test_pub_msg(target_node, topic_name, msg, 10);
  }

  template <typename MessageType>
  void test_pub_msg(
    ros::NodeHandle & target_node, const std::string & topic_name, const MessageType & msg,
    const uint32_t queue_size)
  {
    if (publishers_.find(topic_name) == publishers_.end()) {
      publishers_[topic_name] = test_node_.advertise<MessageType>(topic_name, queue_size);
    }
    publishers_.at(topic_name).publish(msg);
    const int repeat_count = 3;
    (void)target_node;
    autoware::test_utils::spinSomeNodes(repeat_count);
    ROS_INFO_STREAM("Published message on topic '" << topic_name << "'");
  }

  template <typename MessageType>
  void set_subscriber(
    const std::string & topic_name,
    const std::function<void(const typename MessageType::ConstPtr)> & callback,
    const uint32_t queue_size = 10)
  {
    if (subscribers_.find(topic_name) == subscribers_.end()) {
      ros::Subscriber subscriber;
      autoware::test_utils::createSubscription<MessageType>(
        test_node_, topic_name, callback, subscriber, queue_size);
      subscribers_[topic_name] = subscriber;
    } else {
      ROS_WARN_STREAM("Subscriber " << topic_name << " already set.");
    }
  }

  /**
   * @brief Publishes a ROS Clock message with the specified time.
   *
   * This function publishes a ROS Clock message with the specified time.
   * Be careful when using this function, as it can affect the behavior of
   * the system under test. Use an isolated ROS master when tests must be
   * separated from the system clock.
   *
   * @param time The time to publish.
   */
  void jump_clock(const ros::Time & time)
  {
    rosgraph_msgs::Clock clock;
    clock.clock = time;
    pub_clock_.publish(clock);
  }

protected:
  // Node must be constructed before publishers which use it.
  ros::NodeHandle test_node_;

  // Publisher
  std::unordered_map<std::string, ros::Publisher> publishers_;
  std::unordered_map<std::string, ros::Subscriber> subscribers_;
  ros::Publisher pub_clock_;
};  // class AutowareTestManager

std::optional<std::string> resolve_pkg_share_uri(const std::string & uri_path);

}  // namespace autoware::test_utils

#endif  // AUTOWARE_TEST_UTILS__AUTOWARE_TEST_UTILS_HPP_
