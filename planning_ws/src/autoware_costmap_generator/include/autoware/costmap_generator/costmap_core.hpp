#pragma once
#include <grid_map_ros/grid_map_ros.hpp>
#include <string>
namespace autoware::costmap_generator {
class CostmapCore { public:
 static grid_map::GridMap initialize(const std::string &, double, double, double, double, double);
 static grid_map::Matrix combine(const grid_map::GridMap &, const std::string &, const std::string &, const std::string &, const std::string &, double);
 static void snapCenterToPose(grid_map::GridMap &, double, double);
};}
