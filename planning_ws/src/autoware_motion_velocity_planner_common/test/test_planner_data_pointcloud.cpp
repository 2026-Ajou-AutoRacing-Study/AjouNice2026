#include <autoware/motion_velocity_planner_common/planner_data.hpp>

#include <gtest/gtest.h>

TEST(MotionVelocityPlannerDataPointcloud, RejectsReadBeforePreprocess)
{
  autoware::motion_velocity_planner::PlannerData::Pointcloud pointcloud;
  EXPECT_THROW(pointcloud.get_filtered_pointcloud_ptr(), std::runtime_error);
  EXPECT_THROW(pointcloud.get_cluster_indices(), std::runtime_error);
}

TEST(MotionVelocityPlannerDataPointcloud, DownsamplesClustersAndExtractsFilteredPoints)
{
  autoware::motion_velocity_planner::PlannerData::PointcloudPreprocessParams parameters;
  parameters.downsample_by_voxel_grid.enable_downsample = true;
  parameters.downsample_by_voxel_grid.voxel_size_x = 0.1;
  parameters.downsample_by_voxel_grid.voxel_size_y = 0.1;
  parameters.downsample_by_voxel_grid.voxel_size_z = 0.1;
  parameters.euclidean_clustering.enable_clustering = true;
  parameters.euclidean_clustering.cluster_tolerance = 0.5;
  parameters.euclidean_clustering.min_cluster_size = 2;
  parameters.euclidean_clustering.max_cluster_size = 10;
  autoware::motion_velocity_planner::PlannerData::Pointcloud pointcloud(parameters);
  pcl::PointCloud<pcl::PointXYZ> input;
  input.push_back({0.0F, 0.0F, 0.0F});
  input.push_back({0.2F, 0.0F, 0.0F});
  input.push_back({5.0F, 0.0F, 0.0F});
  input.push_back({5.2F, 0.0F, 0.0F});
  pointcloud.preprocess_pointcloud(input);

  const auto filtered = pointcloud.get_filtered_pointcloud_ptr();
  const auto clusters = pointcloud.get_cluster_indices();
  EXPECT_EQ(filtered->size(), 4U);
  EXPECT_EQ(clusters.size(), 2U);
  EXPECT_EQ(pointcloud.extract_clustered_points().size(), 4U);
}

TEST(MotionVelocityPlannerDataPointcloud, FiltersPointsOutsideTrajectoryPolygons)
{
  autoware::motion_velocity_planner::PlannerData::PointcloudPreprocessParams parameters;
  parameters.filter_by_trajectory_polygon.enable_multi_polygon_filtering = true;
  parameters.filter_by_trajectory_polygon.lateral_margin = 0.0;
  parameters.filter_by_trajectory_polygon.min_trajectory_length = 20.0;
  autoware::motion_velocity_planner::PlannerData::Pointcloud pointcloud(parameters);
  std::vector<autoware_planning_msgs::TrajectoryPoint> trajectory(3);
  for (size_t i = 0; i < trajectory.size(); ++i) {
    trajectory.at(i).pose.position.x = static_cast<double>(i * 5U);
    trajectory.at(i).pose.orientation.w = 1.0;
    trajectory.at(i).longitudinal_velocity_mps = 2.0F;
  }
  nav_msgs::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.vehicle_width_m = 2.0;
  vehicle.max_longitudinal_offset_m = 2.0;
  vehicle.rear_overhang_m = 1.0;
  vehicle.wheel_base_m = 2.5;
  pcl::PointCloud<pcl::PointXYZ> input;
  input.push_back({5.0F, 0.0F, 0.0F});
  input.push_back({5.0F, 4.0F, 0.0F});

  pointcloud.preprocess_pointcloud(input, trajectory, odometry, 0.0, vehicle, 5.0, 1.57, 2.0, 0.0, false, 0.0);

  ASSERT_EQ(pointcloud.get_filtered_pointcloud_ptr()->size(), 1U);
  EXPECT_NEAR(pointcloud.get_filtered_pointcloud_ptr()->front().y, 0.0F, 1e-6F);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
