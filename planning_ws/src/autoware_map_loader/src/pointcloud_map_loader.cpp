#include "autoware_map_loader/pointcloud_map_loader.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <algorithm>
#include <cmath>

namespace autoware::map_loader
{
namespace fs = std::filesystem;

// Shared metadata/overlap helpers are provided by the original full-source library.
sensor_msgs::PointCloud2 load_pcd_files(const std::vector<std::string> & paths, float leaf_size)
{
  sensor_msgs::PointCloud2 whole;
  for (const auto & path : paths) {
    sensor_msgs::PointCloud2 part;
    if (pcl::io::loadPCDFile(path, part) == -1) { ROS_ERROR_STREAM("PCD load failed: " << path); continue; }
    if (leaf_size > 0.0F) {
      pcl::PointCloud<pcl::PointXYZ> in, out;
      pcl::fromROSMsg(part, in);
      pcl::VoxelGrid<pcl::PointXYZ> filter;
      filter.setInputCloud(in.makeShared()); filter.setLeafSize(leaf_size, leaf_size, leaf_size); filter.filter(out);
      pcl::toROSMsg(out, part);
    }
    if (whole.width == 0) whole = part;
    else { whole.width += part.width; whole.row_step += part.row_step; whole.data.insert(whole.data.end(), part.data.begin(), part.data.end()); }
  }
  whole.header.frame_id = "map";
  return whole;
}

PointCloudMapLoader::PointCloudMapLoader(ros::NodeHandle nh, ros::NodeHandle pnh)
{
  std::vector<std::string> input_paths;
  std::string metadata_path;
  bool whole, downsampled, partial_enabled, selected_enabled;
  double leaf_size;
  pnh.param("pcd_paths_or_directory", input_paths, std::vector<std::string>{});
  pnh.param<std::string>("pcd_metadata_path", metadata_path, "");
  pnh.param("enable_whole_load", whole, false); pnh.param("enable_downsampled_whole_load", downsampled, false);
  pnh.param("enable_partial_load", partial_enabled, false); pnh.param("enable_selected_load", selected_enabled, false);
  pnh.param("leaf_size", leaf_size, 1.0);
  std::vector<std::string> paths;
  for (const auto & input : input_paths) {
    if (fs::is_regular_file(input) && (fs::path(input).extension() == ".pcd" || fs::path(input).extension() == ".PCD")) paths.push_back(input);
    if (fs::is_directory(input)) for (const auto & file : fs::directory_iterator(input)) if (file.path().extension() == ".pcd" || file.path().extension() == ".PCD") paths.push_back(file.path().string());
  }
  if (fs::exists(metadata_path)) { std::set<std::string> missing; metadata_ = replace_with_absolute_path(load_pcd_metadata(metadata_path), paths, missing); if (!missing.empty()) throw std::runtime_error("Missing PCD segments"); }
  else if (paths.size() == 1) { pcl::PointCloud<pcl::PointXYZ> pcd; if (pcl::io::loadPCDFile(paths.front(), pcd) == -1) throw std::runtime_error("PCD load failed"); PCDFileMetadata m; pcl::getMinMax3D(pcd, m.min, m.max); metadata_[paths.front()] = m; }
  else if (!paths.empty()) throw std::runtime_error("PCD metadata file not found");
  if (whole) { whole_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/output/pointcloud_map", 1, true); whole_pub_.publish(load_pcd_files(paths)); }
  if (downsampled) { downsample_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/output/debug/downsampled_pointcloud_map", 1, true); downsample_pub_.publish(load_pcd_files(paths, static_cast<float>(leaf_size))); }
  if (partial_enabled) partial_srv_ = nh.advertiseService("/service/get_partial_pcd_map", &PointCloudMapLoader::partial, this);
  differential_srv_ = nh.advertiseService("/service/get_differential_pcd_map", &PointCloudMapLoader::differential, this);
  if (selected_enabled) { selected_srv_ = nh.advertiseService("/service/get_selected_pcd_map", &PointCloudMapLoader::selected, this); metadata_pub_ = nh.advertise<autoware_map_msgs::PointCloudMapMetaData>("/output/pointcloud_map_metadata", 1, true); autoware_map_msgs::PointCloudMapMetaData message; message.header.frame_id = "map"; message.header.stamp = ros::Time::now(); for (const auto & item : metadata_) { autoware_map_msgs::PointCloudMapCellMetaDataWithID value; value.cell_id = item.first; value.metadata.min_x=item.second.min.x; value.metadata.min_y=item.second.min.y; value.metadata.max_x=item.second.max.x; value.metadata.max_y=item.second.max.y; message.metadata_list.push_back(value); } metadata_pub_.publish(message); }
}

void PointCloudMapLoader::append_cell(const std::string & path, autoware_map_msgs::PointCloudMapCellWithID & cell) const
{
  pcl::io::loadPCDFile(path, cell.pointcloud); cell.cell_id = path;
  const auto & meta = metadata_.at(path); cell.metadata.min_x=meta.min.x; cell.metadata.min_y=meta.min.y; cell.metadata.max_x=meta.max.x; cell.metadata.max_y=meta.max.y;
}
bool PointCloudMapLoader::partial(autoware_map_msgs::GetPartialPointCloudMap::Request & req, autoware_map_msgs::GetPartialPointCloudMap::Response & res)
{
  for (const auto & item : metadata_) if (cylinder_and_box_overlap_exists(req.area.center_x, req.area.center_y, req.area.radius, item.second.min, item.second.max)) { autoware_map_msgs::PointCloudMapCellWithID cell; append_cell(item.first, cell); res.new_pointcloud_with_ids.push_back(cell); }
  res.header.frame_id="map"; return true;
}
bool PointCloudMapLoader::differential(autoware_map_msgs::GetDifferentialPointCloudMap::Request & req, autoware_map_msgs::GetDifferentialPointCloudMap::Response & res)
{
  for (const auto & item : metadata_) { const bool selected=cylinder_and_box_overlap_exists(req.area.center_x,req.area.center_y,req.area.radius,item.second.min,item.second.max); const bool cached=std::find(req.cached_ids.begin(),req.cached_ids.end(),item.first)!=req.cached_ids.end(); if(selected&&!cached){autoware_map_msgs::PointCloudMapCellWithID cell;append_cell(item.first,cell);res.new_pointcloud_with_ids.push_back(cell);} }
  for(const auto & id:req.cached_ids) if(metadata_.find(id)==metadata_.end() || !cylinder_and_box_overlap_exists(req.area.center_x,req.area.center_y,req.area.radius,metadata_.at(id).min,metadata_.at(id).max)) res.ids_to_remove.push_back(id);
  res.header.frame_id="map"; return true;
}
bool PointCloudMapLoader::selected(autoware_map_msgs::GetSelectedPointCloudMap::Request & req, autoware_map_msgs::GetSelectedPointCloudMap::Response & res)
{
  for(const auto & id:req.cell_ids) if(metadata_.count(id)){autoware_map_msgs::PointCloudMapCellWithID cell;append_cell(id,cell);res.new_pointcloud_with_ids.push_back(cell);} res.header.frame_id="map"; return true;
}
}  // namespace autoware::map_loader
