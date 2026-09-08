// Copyright 2020 Tier IV, Inc.
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

#include "autoware_utils_pcl/transforms.hpp"

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <thread>

namespace
{
pcl::PointXYZI make_point_xyzi(float x, float y, float z, float intensity)
{
  pcl::PointXYZI point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  return point;
}
}  // namespace

TEST(system, transform_point_cloud)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(make_point_xyzi(10.055880F, -42.758892F, -10.636949F, 4.0F));
  cloud.push_back(make_point_xyzi(23.282284F, -29.485722F, -11.468469F, 5.0F));

  Eigen::Matrix<float, 4, 4> transform;
  transform << 0.834513, -0.550923, -0.008474, 89571.148438, 0.550986, 0.834372, 0.015428,
    42301.179688, -0.001429, -0.017543, 0.999845, -3.157415, 0.000000, 0.000000, 0.000000, 1.000000;

  pcl::PointCloud<pcl::PointXYZI> cloud_transformed;
  autoware_utils_pcl::transform_pointcloud(cloud, cloud_transformed, transform);

  const auto pt1_gt = make_point_xyzi(89603.187500F, 42270.878906F, -13.056946F, 4.0F);

  constexpr float float_error = 0.0001;
  EXPECT_NEAR(cloud_transformed[0].x, pt1_gt.x, float_error);
  EXPECT_NEAR(cloud_transformed[0].y, pt1_gt.y, float_error);
  EXPECT_NEAR(cloud_transformed[0].z, pt1_gt.z, float_error);
  EXPECT_EQ(cloud_transformed[0].intensity, pt1_gt.intensity);
}

TEST(system, empty_point_cloud)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;

  Eigen::Matrix<float, 4, 4> transform;
  transform << 0.834513, -0.550923, -0.008474, 89571.148438, 0.550986, 0.834372, 0.015428,
    42301.179688, -0.001429, -0.017543, 0.999845, -3.157415, 0.000000, 0.000000, 0.000000, 1.000000;

  pcl::PointCloud<pcl::PointXYZI> cloud_transformed;

  EXPECT_NO_THROW(autoware_utils_pcl::transform_pointcloud(cloud, cloud_transformed, transform));
  EXPECT_NO_FATAL_FAILURE(
    autoware_utils_pcl::transform_pointcloud(cloud, cloud_transformed, transform));
  EXPECT_EQ(cloud_transformed.size(), 0ul);
}
