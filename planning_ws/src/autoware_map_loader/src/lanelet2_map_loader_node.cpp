#include "autoware_map_loader/lanelet2_map_loader.hpp"

#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <ros/ros.h>

#include <cctype>
#include <stdexcept>
#include <string>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "lanelet2_map_loader");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  std::string map_path;
  double centerline_resolution;
  bool use_waypoints;
  bool allow_unsupported_version;
  pnh.param<std::string>("lanelet2_map_path", map_path, "");
  pnh.param("center_line_resolution", centerline_resolution, 5.0);
  pnh.param("use_waypoints", use_waypoints, true);
  pnh.param("allow_unsupported_version", allow_unsupported_version, true);
  const auto publisher = nh.advertise<autoware_map_msgs::LaneletMapBin>("/map/vector_map", 1, true);
  const auto callback = [&](const autoware_map_msgs::MapProjectorInfo::ConstPtr & info) {
    try {
      const auto map = autoware::map_loader::load_lanelet2_map(map_path, *info);
      if (!map) throw std::runtime_error("failed to load lanelet2 map");
      std::string format_version, map_version;
      lanelet::io_handlers::AutowareOsmParser::parseVersions(map_path, &format_version, &map_version);
      if ((format_version.empty() || format_version == "null" || !std::isdigit(format_version[0])) && !allow_unsupported_version) {
        throw std::invalid_argument("unsupported map format version");
      }
      if (use_waypoints) {
        lanelet::utils::overwriteLaneletsCenterlineWithWaypoints(map, centerline_resolution, false);
      } else {
        lanelet::utils::overwriteLaneletsCenterline(map, centerline_resolution, false);
      }
      publisher.publish(autoware::map_loader::create_map_bin_msg(map, map_path));
      ROS_INFO_STREAM("published lanelet2 map: " << map_path);
    } catch (const std::exception & error) {
      ROS_ERROR_STREAM("lanelet2_map_loader: " << error.what());
    }
  };
  const auto subscription = nh.subscribe<autoware_map_msgs::MapProjectorInfo>(
    "/map/map_projector_info", 1, callback);
  ros::spin();
  return 0;
}
