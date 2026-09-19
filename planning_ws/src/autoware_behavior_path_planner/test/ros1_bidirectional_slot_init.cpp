#include <autoware/behavior_path_planner/behavior_path_planner_core.hpp>

#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "aw149_bpp_bidirectional_slot_init");
  ros::NodeHandle private_node("aw149_bpp_bidirectional_slot_init");
  private_node.setParam("enable_bidirectional_traffic_module", true);
  private_node.setParam("side_shift/min_distance_to_start_shifting", 1.0);
  private_node.setParam("side_shift/min_shifting_distance", 2.0);
  private_node.setParam("side_shift/min_shifting_speed", 1.0);
  autoware::behavior_path_planner::BehaviorPathPlannerCore core(&private_node);
  return core.isInitialized() && core.bidirectionalTrafficEnabled() ? 0 : 1;
}
