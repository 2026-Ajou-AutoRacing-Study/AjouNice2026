#ifndef AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__OCCUPANCY_GRID_BASED_COLLISION_DETECTOR__OCCUPANCY_GRID_BASED_COLLISION_DETECTOR_HPP_
#define AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__OCCUPANCY_GRID_BASED_COLLISION_DETECTOR__OCCUPANCY_GRID_BASED_COLLISION_DETECTOR_HPP_

#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

#include <vector>

namespace autoware::behavior_path_planner
{
int discretize_angle(double theta, int theta_size);

struct IndexXYT { int x; int y; int theta; };
struct IndexXY { int x; int y; };

IndexXYT pose2index(
  const nav_msgs::OccupancyGrid & costmap, const geometry_msgs::Pose & pose_local, int theta_size);
geometry_msgs::Pose index2pose(
  const nav_msgs::OccupancyGrid & costmap, const IndexXYT & index, int theta_size);
geometry_msgs::Pose global2local(
  const nav_msgs::OccupancyGrid & costmap, const geometry_msgs::Pose & pose_global);

struct VehicleShape { double length{}; double width{}; double base2back{}; };
struct OccupancyGridMapParam {
  VehicleShape vehicle_shape;
  int theta_size{0};
  int obstacle_threshold{0};
};

class OccupancyGridBasedCollisionDetector
{
public:
  void setParam(const OccupancyGridMapParam & param) { param_ = param; }
  [[nodiscard]] OccupancyGridMapParam getParam() const { return param_; }
  void setMap(const nav_msgs::OccupancyGrid & costmap);
  [[nodiscard]] nav_msgs::OccupancyGrid getMap() const { return costmap_; }
  [[nodiscard]] bool hasObstacleOnPath(
    const autoware_internal_planning_msgs::PathWithLaneId & path, bool check_out_of_range) const;
  [[nodiscard]] bool detectCollision(const IndexXYT & base_index, bool check_out_of_range) const;

private:
  void computeCollisionIndexes(int theta_index, std::vector<IndexXY> * indexes);
  [[nodiscard]] bool isOutOfRange(const IndexXYT & index) const;
  [[nodiscard]] bool isObstacle(const IndexXYT & index) const;

  OccupancyGridMapParam param_;
  nav_msgs::OccupancyGrid costmap_;
  std::vector<std::vector<IndexXY>> collision_indexes_table_;
  std::vector<std::vector<bool>> obstacle_table_;
};
}  // namespace autoware::behavior_path_planner

#endif
