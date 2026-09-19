#include "autoware/planning_validator/trajectory_checker_core.hpp"

#include <ros/ros.h>

namespace
{
class TrajectoryCheckerStaticNode
{
public:
  TrajectoryCheckerStaticNode()
  : private_nh_("~"), checker_(parameters())
  {
    status_pub_ = private_nh_.advertise<autoware_planning_validator::PlanningValidatorStatus>(
      "output/status", 1, false);
    trajectory_sub_ = private_nh_.subscribe(
      "input/trajectory", 1, &TrajectoryCheckerStaticNode::on_trajectory, this);
  }

private:
  autoware::planning_validator::TrajectoryCheckerCoreParameters parameters()
  {
    autoware::planning_validator::TrajectoryCheckerCoreParameters result;
    private_nh_.param("wheel_base_m", result.wheel_base_m, result.wheel_base_m);
    return result;
  }
  void on_trajectory(const autoware_planning_msgs::Trajectory::ConstPtr & trajectory)
  {
    status_pub_.publish(checker_.validate(*trajectory));
  }

  ros::NodeHandle private_nh_;
  autoware::planning_validator::TrajectoryCheckerCore checker_;
  ros::Publisher status_pub_;
  ros::Subscriber trajectory_sub_;
};
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "trajectory_checker_static");
  TrajectoryCheckerStaticNode node;
  ros::spin();
  return 0;
}
