#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__INTERFACE_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__INTERFACE_HPP_

#include <ros/ros.h>

#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace autoware::component_interface_utils
{

template <class T>
inline std::string to_yaml(const T &) { return ""; }

class ClientDispatchWorker
{
public:
  ClientDispatchWorker() : thread_([this] { run(); }) {}
  ClientDispatchWorker(const ClientDispatchWorker &) = delete;
  ClientDispatchWorker & operator=(const ClientDispatchWorker &) = delete;
  ~ClientDispatchWorker()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_one();
    thread_.join();
  }

  void dispatch(std::function<void()> task)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      tasks_.push_back(std::move(task));
    }
    condition_.notify_one();
  }

private:
  void run()
  {
    for (;;) {
      std::function<void()> task;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return stopping_ || !tasks_.empty(); });
        if (stopping_ && tasks_.empty()) {
          return;
        }
        task = std::move(tasks_.front());
        tasks_.pop_front();
      }
      task();
    }
  }

  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<std::function<void()>> tasks_;
  bool stopping_{false};
  std::thread thread_;
};

struct NodeInterface
{
  using SharedPtr = std::shared_ptr<NodeInterface>;
  explicit NodeInterface(const ros::NodeHandle & node)
  : node(node)
  {
    node_name = ros::this_node::getName();
  }

  void log(uint8_t type, const std::string & name, const std::string & yaml = "") const
  {
    ROS_DEBUG_STREAM("component interface event=" << static_cast<int>(type) << " name=" << name << " yaml=" << yaml);
  }

  void dispatch_client(std::function<void()> task) { client_worker_.dispatch(std::move(task)); }

  ros::NodeHandle node;
  std::string node_name;

private:
  ClientDispatchWorker client_worker_;
};

}  // namespace autoware::component_interface_utils

#endif
