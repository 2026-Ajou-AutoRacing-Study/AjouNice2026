#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace
{

std::string trim(const std::string& value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos)
  {
    return std::string();
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::vector<std::string> splitCsv(const std::string& line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ','))
  {
    fields.push_back(trim(field));
  }
  if (!line.empty() && line.back() == ',')
  {
    fields.emplace_back();
  }
  return fields;
}

double parseFiniteDouble(const std::string& value, const std::string& field, const std::size_t line_number)
{
  std::size_t consumed = 0;
  double result = 0.0;
  try
  {
    result = std::stod(value, &consumed);
  }
  catch (const std::exception&)
  {
    throw std::runtime_error("line " + std::to_string(line_number) + " invalid " + field + "='" + value + "'");
  }
  if (consumed != value.size() || !std::isfinite(result))
  {
    throw std::runtime_error("line " + std::to_string(line_number) + " non-finite or malformed " + field);
  }
  return result;
}

std::size_t parseIndex(const std::string& value, const std::size_t line_number)
{
  std::size_t consumed = 0;
  unsigned long result = 0;
  try
  {
    result = std::stoul(value, &consumed);
  }
  catch (const std::exception&)
  {
    throw std::runtime_error("line " + std::to_string(line_number) + " invalid index='" + value + "'");
  }
  if (consumed != value.size())
  {
    throw std::runtime_error("line " + std::to_string(line_number) + " malformed index");
  }
  return static_cast<std::size_t>(result);
}

long long parseInteger(const std::string& value, const std::string& field, const std::size_t line_number)
{
  std::size_t consumed = 0;
  long long result = 0;
  try
  {
    result = std::stoll(value, &consumed);
  }
  catch (const std::exception&)
  {
    throw std::runtime_error("line " + std::to_string(line_number) + " invalid " + field + "='" + value + "'");
  }
  if (consumed != value.size())
  {
    throw std::runtime_error("line " + std::to_string(line_number) + " malformed " + field);
  }
  return result;
}

}  // namespace

class TrajectoryPublisher
{
public:
  TrajectoryPublisher() : private_nh_("~")
  {
    private_nh_.param<std::string>("trajectory_topic", trajectory_topic_, "/control/trajectory");
    private_nh_.param<std::string>("debug_path_topic", debug_path_topic_, "/control/debug/generator/path");
    private_nh_.param<std::string>("debug_marker_topic", debug_marker_topic_,
                                   "/control/debug/generator/markers");
    private_nh_.param<std::string>("frame_id", frame_id_, "map");
    private_nh_.param("publish_rate_hz", publish_rate_hz_, 50.0);
    if (!private_nh_.getParam("trajectory_csv", trajectory_csv_))
    {
      throw std::runtime_error("~trajectory_csv is required");
    }
    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0)
    {
      throw std::runtime_error("~publish_rate_hz must be finite and positive");
    }
    if (frame_id_.empty())
    {
      throw std::runtime_error("~frame_id must not be empty");
    }
    if (trajectory_topic_.empty() || debug_path_topic_.empty() || debug_marker_topic_.empty())
    {
      throw std::runtime_error("publisher topic parameters must not be empty");
    }

    const std::string package_path = ros::package::getPath("trajectory_generator");
    if (package_path.empty())
    {
      throw std::runtime_error("cannot resolve trajectory_generator package path");
    }
    if (trajectory_csv_.empty() || trajectory_csv_.front() == '/' || trajectory_csv_.find("..") != std::string::npos)
    {
      throw std::runtime_error("~trajectory_csv must be a package-relative path without '..'");
    }
    csv_path_ = package_path + "/" + trajectory_csv_;
    loadCsv();

    trajectory_publisher_ = nh_.advertise<autoware_planning_msgs::Trajectory>(trajectory_topic_, 1, true);
    path_publisher_ = nh_.advertise<nav_msgs::Path>(debug_path_topic_, 1, true);
    marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(debug_marker_topic_, 1, true);
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_hz_), &TrajectoryPublisher::publish, this);

    ROS_INFO_STREAM("trajectory_publisher loaded " << trajectory_.points.size() << " points from " << csv_path_
                                                     << " -> " << trajectory_topic_ << " at " << publish_rate_hz_
                                                     << " Hz frame=" << frame_id_);
  }

private:
  void loadCsv()
  {
    std::ifstream stream(csv_path_);
    if (!stream)
    {
      throw std::runtime_error("cannot open trajectory CSV: " + csv_path_);
    }
    std::string line;
    if (!std::getline(stream, line))
    {
      throw std::runtime_error("trajectory CSV is empty: " + csv_path_);
    }
    const auto header = splitCsv(line);
    std::unordered_map<std::string, std::size_t> columns;
    for (std::size_t index = 0; index < header.size(); ++index)
    {
      if (header[index].empty() || columns.count(header[index]) != 0U)
      {
        throw std::runtime_error("trajectory CSV has an empty or duplicate header field");
      }
      columns[header[index]] = index;
    }
    const std::vector<std::string> required = { "index", "x", "y", "z", "yaw", "velocity_mps",
                                                "acceleration_mps2", "lanelet_id", "route_order" };
    for (const auto& field : required)
    {
      if (columns.count(field) == 0U)
      {
        throw std::runtime_error("trajectory CSV missing required column: " + field);
      }
    }

    trajectory_.header.frame_id = frame_id_;
    debug_path_.header.frame_id = frame_id_;
    std::size_t expected_index = 0;
    std::size_t line_number = 1;
    double accumulated_time = 0.0;
    autoware_planning_msgs::TrajectoryPoint previous;
    bool have_previous = false;
    while (std::getline(stream, line))
    {
      ++line_number;
      if (trim(line).empty())
      {
        continue;
      }
      const auto fields = splitCsv(line);
      if (fields.size() != header.size())
      {
        throw std::runtime_error("line " + std::to_string(line_number) + " has wrong column count");
      }
      const auto value = [&](const std::string& name) -> const std::string& { return fields.at(columns.at(name)); };
      const std::size_t index = parseIndex(value("index"), line_number);
      if (index != expected_index)
      {
        throw std::runtime_error("line " + std::to_string(line_number) + " index is not contiguous from zero");
      }
      ++expected_index;
      (void)parseInteger(value("lanelet_id"), "lanelet_id", line_number);
      (void)parseInteger(value("route_order"), "route_order", line_number);

      autoware_planning_msgs::TrajectoryPoint point;
      point.pose.position.x = parseFiniteDouble(value("x"), "x", line_number);
      point.pose.position.y = parseFiniteDouble(value("y"), "y", line_number);
      point.pose.position.z = parseFiniteDouble(value("z"), "z", line_number);
      const double yaw = parseFiniteDouble(value("yaw"), "yaw", line_number);
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, yaw);
      point.pose.orientation = tf2::toMsg(orientation);
      point.longitudinal_velocity_mps =
          static_cast<float>(parseFiniteDouble(value("velocity_mps"), "velocity_mps", line_number));
      point.acceleration_mps2 =
          static_cast<float>(parseFiniteDouble(value("acceleration_mps2"), "acceleration_mps2", line_number));
      if (point.longitudinal_velocity_mps < 0.0F)
      {
        throw std::runtime_error("line " + std::to_string(line_number) + " has negative target velocity");
      }
      if (have_previous)
      {
        const double dx = point.pose.position.x - previous.pose.position.x;
        const double dy = point.pose.position.y - previous.pose.position.y;
        const double dz = point.pose.position.z - previous.pose.position.z;
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double average_speed = std::max(
            0.1, 0.5 * (static_cast<double>(point.longitudinal_velocity_mps) +
                        static_cast<double>(previous.longitudinal_velocity_mps)));
        accumulated_time += distance / average_speed;
      }
      point.time_from_start = ros::Duration(accumulated_time);
      point.lateral_velocity_mps = 0.0F;
      point.heading_rate_rps = 0.0F;
      point.front_wheel_angle_rad = 0.0F;
      point.rear_wheel_angle_rad = 0.0F;
      trajectory_.points.push_back(point);

      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = frame_id_;
      pose.pose = point.pose;
      debug_path_.poses.push_back(pose);
      previous = point;
      have_previous = true;
    }
    if (trajectory_.points.size() < 2U)
    {
      throw std::runtime_error("trajectory CSV must contain at least two points");
    }
  }

  visualization_msgs::Marker makeSphere(const int id, const geometry_msgs::Point& point, const float red,
                                         const float green, const ros::Time& stamp) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "trajectory_endpoints";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = point;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = 0.1F;
    marker.color.a = 1.0F;
    return marker;
  }

  visualization_msgs::MarkerArray makeMarkers(const ros::Time& stamp) const
  {
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker line;
    line.header.frame_id = frame_id_;
    line.header.stamp = stamp;
    line.ns = "trajectory";
    line.id = 0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.15;
    line.color.r = 0.1F;
    line.color.g = 0.6F;
    line.color.b = 1.0F;
    line.color.a = 0.9F;
    line.points.reserve(trajectory_.points.size());
    for (const auto& point : trajectory_.points)
    {
      line.points.push_back(point.pose.position);
    }
    array.markers.push_back(line);
    array.markers.push_back(makeSphere(1, trajectory_.points.front().pose.position, 0.1F, 1.0F, stamp));
    array.markers.push_back(makeSphere(2, trajectory_.points.back().pose.position, 1.0F, 0.1F, stamp));
    return array;
  }

  void publish(const ros::TimerEvent&)
  {
    const ros::Time stamp = ros::Time::now();
    trajectory_.header.stamp = stamp;
    debug_path_.header.stamp = stamp;
    for (auto& pose : debug_path_.poses)
    {
      pose.header.stamp = stamp;
    }
    trajectory_publisher_.publish(trajectory_);
    path_publisher_.publish(debug_path_);
    marker_publisher_.publish(makeMarkers(stamp));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher trajectory_publisher_;
  ros::Publisher path_publisher_;
  ros::Publisher marker_publisher_;
  ros::Timer timer_;
  std::string trajectory_topic_;
  std::string debug_path_topic_;
  std::string debug_marker_topic_;
  std::string frame_id_;
  std::string trajectory_csv_;
  std::string csv_path_;
  double publish_rate_hz_{ 50.0 };
  autoware_planning_msgs::Trajectory trajectory_;
  nav_msgs::Path debug_path_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_publisher");
  try
  {
    TrajectoryPublisher publisher;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("trajectory_publisher: %s", error.what());
    return 1;
  }
  return 0;
}
