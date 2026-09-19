#include "autoware_map_loader/pointcloud_map_loader.hpp"

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <fstream>
#include "full_original/native_test_support.hpp"

namespace
{
std::string temp(const std::string & name) { return (owned_map_test_dir() / ("autoware_map_loader_" + name)).string(); }
void write_pcd(const std::string & path)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back({0.0F, 0.0F, 0.0F}); cloud.push_back({1.0F, 2.0F, 3.0F}); cloud.push_back({2.0F, 4.0F, 6.0F});
  cloud.width=cloud.size(); cloud.height=1;
  ASSERT_EQ(pcl::io::savePCDFileASCII(path, cloud), 0);
}
}  // namespace

TEST(PointCloudMapLoader, CylinderBoxOverlapPreservesThreeUpstreamCases)
{
  pcl::PointXYZ min; min.x=-1; min.y=-1; pcl::PointXYZ max; max.x=1; max.y=1;
  EXPECT_FALSE(autoware::map_loader::cylinder_and_box_overlap_exists(5, 0, 3.9, min, max));
  EXPECT_TRUE(autoware::map_loader::cylinder_and_box_overlap_exists(0, 0, 5, min, max));
  min.z=-100; max.z=-99;
  EXPECT_TRUE(autoware::map_loader::cylinder_and_box_overlap_exists(0, 0, 5, min, max));
}

TEST(PointCloudMapLoader, MetadataAndAbsolutePathResolution)
{
  const auto yaml=temp("metadata.yaml");
  std::ofstream file(yaml); file << "file1.pcd: [1, 2]\nfile2.pcd: [3, 4]\nx_resolution: 5\ny_resolution: 6\n"; file.close();
  const auto metadata=autoware::map_loader::load_pcd_metadata(yaml);
  EXPECT_FLOAT_EQ(metadata.at("file1.pcd").max.x, 6); EXPECT_FLOAT_EQ(metadata.at("file2.pcd").max.y, 10);
  std::set<std::string> missing;
  const auto absolute=autoware::map_loader::replace_with_absolute_path(metadata,{"/tmp/file1.pcd","/tmp/file2.pcd"},missing);
  EXPECT_TRUE(missing.empty()); EXPECT_EQ(absolute.size(), 2U); EXPECT_EQ(absolute.begin()->first, "/tmp/file1.pcd");
}

TEST(PointCloudMapLoader, WholePcdLoadPreservesPointsAndMapFrame)
{
  const auto path=temp("whole.pcd"); write_pcd(path);
  const auto actual=autoware::map_loader::load_pcd_files({path});
  pcl::PointCloud<pcl::PointXYZ> cloud; pcl::fromROSMsg(actual, cloud);
  ASSERT_EQ(cloud.size(), 3U); EXPECT_EQ(actual.header.frame_id, "map"); EXPECT_FLOAT_EQ(cloud.points[2].z, 6.0F);
}

int main(int argc, char ** argv) { ::testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
