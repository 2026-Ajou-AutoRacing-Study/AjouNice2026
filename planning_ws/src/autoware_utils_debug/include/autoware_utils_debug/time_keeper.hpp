#pragma once

#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_internal_debug_msgs/ProcessingTimeNode.h>
#include <autoware_internal_debug_msgs/ProcessingTimeTree.h>

#include <ros/ros.h>

#include <functional>
#include <memory>
#include <ostream>
#include <string>
#include <thread>
#include <vector>

namespace autoware_utils_debug
{
class ProcessingTimeNode : public std::enable_shared_from_this<ProcessingTimeNode>
{
public:
  explicit ProcessingTimeNode(const std::string & name);
  std::shared_ptr<ProcessingTimeNode> add_child(const std::string & name);
  std::string to_string() const;
  autoware_internal_debug_msgs::ProcessingTimeTree to_msg() const;
  std::weak_ptr<ProcessingTimeNode> get_parent_node() const;
  std::vector<std::shared_ptr<ProcessingTimeNode>> get_child_nodes() const;
  void set_time(double processing_time);
  void set_comment(const std::string & comment);
  std::string get_name() const;
private:
  const std::string name_;
  double processing_time_{0.0};
  std::string comment_;
  std::weak_ptr<ProcessingTimeNode> parent_node_;
  std::vector<std::shared_ptr<ProcessingTimeNode>> child_nodes_;
};

using ProcessingTimeDetail = autoware_internal_debug_msgs::ProcessingTimeTree;

class TimeKeeper
{
public:
  template <typename... Reporters> explicit TimeKeeper(Reporters... reporters)
  { (add_reporter(std::move(reporters)), ...); }
  void add_reporter(std::ostream * os);
  void add_reporter(const ros::Publisher & publisher);
  void start_track(const std::string & func_name);
  void end_track(const std::string & func_name);
  void comment(const std::string & comment);
private:
  void report();
  std::shared_ptr<ProcessingTimeNode> current_time_node_;
  std::shared_ptr<ProcessingTimeNode> root_node_;
  std::thread::id root_node_thread_id_;
  autoware_utils_system::StopWatch<std::chrono::milliseconds, std::chrono::microseconds,
    std::chrono::steady_clock> stop_watch_;
  std::vector<std::function<void(const std::shared_ptr<ProcessingTimeNode> &)>> reporters_;
};

class ScopedTimeTrack
{
public:
  ScopedTimeTrack(const std::string & func_name, TimeKeeper & time_keeper);
  ScopedTimeTrack(const ScopedTimeTrack &) = delete;
  ScopedTimeTrack & operator=(const ScopedTimeTrack &) = delete;
  ~ScopedTimeTrack();
private:
  const std::string func_name_;
  TimeKeeper & time_keeper_;
};
}
