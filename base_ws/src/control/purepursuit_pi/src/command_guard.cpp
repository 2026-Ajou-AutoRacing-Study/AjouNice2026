#include "purepursuit_pi/controller.hpp"

namespace purepursuit_pi {
void Controller::publishSafeStop(const std::string& reason, const ros::Time& stamp) {
  morai_msgs::CtrlCmd command;
  command.longlCmdType = 1;
  command.accel = 0.0; command.brake = safe_stop_brake_;
  command.front_steer = 0.0; command.rear_steer = 0.0;
  command.velocity = 0.0; command.acceleration = 0.0;
  command_pub_.publish(command);
  std_msgs::String state; state.data = "SAFE_STOP: " + reason; state_pub_.publish(state);
  if (last_state_ != state.data) ROS_WARN("purepursuit_pi %s", state.data.c_str());
  last_state_ = state.data;
  integral_ = 0.0;
  last_steer_ = 0.0;
  (void)stamp;
}
}  // namespace purepursuit_pi
