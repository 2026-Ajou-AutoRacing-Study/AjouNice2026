#pragma once
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <functional>

namespace autoware::behavior_path_planner
{
// One queue and worker per source MutuallyExclusive callback group. Owned by
// the module, stopped and joined before the callback's referenced data dies.
class Ros1OwnedGoalTimer
{
public:
  Ros1OwnedGoalTimer(ros::NodeHandle & node, std::function<void()> callback)
  : spinner_(1, &queue_)
  {
    ros::TimerOptions options(ros::Duration(1.0),
      [callback = std::move(callback)](const ros::TimerEvent &) { callback(); },
      &queue_, false, false);
    timer_ = node.createTimer(options);
  }
  ~Ros1OwnedGoalTimer() { stop(); }
  void start() { timer_.start(); spinner_.start(); }
  void stop() { timer_.stop(); queue_.disable(); spinner_.stop(); queue_.clear(); }
private:
  ros::CallbackQueue queue_;
  ros::AsyncSpinner spinner_;
  ros::Timer timer_;
};
}
