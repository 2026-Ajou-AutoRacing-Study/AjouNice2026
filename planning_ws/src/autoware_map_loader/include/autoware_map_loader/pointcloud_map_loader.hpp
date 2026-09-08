#ifndef AUTOWARE_MAP_LOADER_POINTCLOUD_MAP_LOADER_HPP_
#define AUTOWARE_MAP_LOADER_POINTCLOUD_MAP_LOADER_HPP_

#include <autoware/map_loader/pointcloud_utils.hpp>
#include <autoware_map_msgs/AreaInfo.h>
#include <autoware_map_msgs/GetDifferentialPointCloudMap.h>
#include <autoware_map_msgs/GetPartialPointCloudMap.h>
#include <autoware_map_msgs/GetSelectedPointCloudMap.h>
#include <autoware_map_msgs/PointCloudMapMetaData.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <map>
#include <set>
#include <string>
#include <vector>

namespace autoware::map_loader
{
std::map<std::string, PCDFileMetadata> load_pcd_metadata(const std::string & path);
std::map<std::string, PCDFileMetadata> replace_with_absolute_path(
  const std::map<std::string, PCDFileMetadata> & metadata, const std::vector<std::string> & paths,
  std::set<std::string> & missing);
bool cylinder_and_box_overlap_exists(double x, double y, double radius, pcl::PointXYZ min, pcl::PointXYZ max);
sensor_msgs::PointCloud2 load_pcd_files(const std::vector<std::string> & paths, float leaf_size = 0.0F);

class PointCloudMapLoader
{
public:
  PointCloudMapLoader(ros::NodeHandle nh, ros::NodeHandle pnh);
private:
  using MetadataMap = std::map<std::string, PCDFileMetadata>;
  MetadataMap metadata_;
  ros::Publisher whole_pub_, downsample_pub_, metadata_pub_;
  ros::ServiceServer partial_srv_, differential_srv_, selected_srv_;
  bool partial(autoware_map_msgs::GetPartialPointCloudMap::Request &, autoware_map_msgs::GetPartialPointCloudMap::Response &);
  bool differential(autoware_map_msgs::GetDifferentialPointCloudMap::Request &, autoware_map_msgs::GetDifferentialPointCloudMap::Response &);
  bool selected(autoware_map_msgs::GetSelectedPointCloudMap::Request &, autoware_map_msgs::GetSelectedPointCloudMap::Response &);
  void append_cell(const std::string & path, autoware_map_msgs::PointCloudMapCellWithID & cell) const;
};
}  // namespace autoware::map_loader

#endif
