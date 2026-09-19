#include <autoware/planning_evaluator/planning_evaluator_node.hpp>
#include <autoware/planning_evaluator/motion_evaluator_node.hpp>

int main(int argc, char ** argv)
{
#ifdef AW149_MOTION_EVALUATOR
  ros::init(argc, argv, "motion_evaluator");
  using Evaluator = planning_diagnostics::MotionEvaluatorNode;
#else
  ros::init(argc, argv, "planning_evaluator");
  using Evaluator = planning_diagnostics::PlanningEvaluatorNode;
#endif
  try {
    Evaluator evaluator(ros::NodeHandle("~"));
    ros::spin();
    return 0;
  } catch (const std::exception & e) {
    ROS_FATAL("Evaluator initialization/runtime failure: %s", e.what());
    return 1;
  }
}
