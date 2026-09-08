// ROS1 transport for the retained original PlanningFactor test.
// The target is the actual installed BPP executable, never a mocked planner.
#pragma once
#include <ros/ros.h>
#include <map>
#include <memory>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <tier4_planning_msgs/LateralOffset.h>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_internal_planning_msgs/Scenario.h>
#include <autoware_adapi_v1_msgs/OperationModeState.h>
#include <ros/package.h>
#include <spawn.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
#include <functional>
#include <thread>
#include <chrono>

extern char ** environ;
namespace autoware::behavior_path_planner {
inline void spinTestFor(double seconds)
{
  const auto end = ros::WallTime::now() + ros::WallDuration(seconds);
  do {
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  } while (ros::WallTime::now() < end);
}

class BehaviorPathPlannerNode {
public:
  explicit BehaviorPathPlannerNode(const std::vector<std::string> & plugins)
  {
    const auto source = std::make_shared<ros::NodeHandle>("/full_goal_test");
    XmlRpc::XmlRpcValue parameters;
    if (!source->getParam("", parameters)) throw std::runtime_error("Missing BPP test parameters");
    ros::NodeHandle target("/behavior_path_planner");
    target.setParam("", parameters);
    target.setParam("launch_modules", plugins);
    // This original fixture requests Goal only, not the legacy built-in SideShift.
    target.setParam("enable_side_shift_module", false);
    // Input freshness is maintained by the test manager's republisher below.
    std::string executable = ros::package::getPath("autoware_behavior_path_planner") +
      "/../../lib/autoware_behavior_path_planner/behavior_path_planner_node";
    if (access(executable.c_str(), X_OK) != 0)
      throw std::runtime_error("Installed BPP executable missing: " + executable);
    char * argv[] = {const_cast<char *>(executable.c_str()),
      const_cast<char *>("__name:=behavior_path_planner"),
      const_cast<char *>("~input/acceleration:=/behavior_path_planner/input/accel"),
      const_cast<char *>("~input/objects:=/behavior_path_planner/input/perception"),
      const_cast<char *>("~input/occupancy_grid:=/behavior_path_planner/input/occupancy_grid_map"),
      const_cast<char *>("~input/operation_mode_state:=/system/operation_mode/state"), nullptr};
    const int error = posix_spawn(&pid_, executable.c_str(), nullptr, nullptr, argv, environ);
    if (error) throw std::runtime_error("Cannot start real BPP process: " + std::to_string(error));
  }
  ~BehaviorPathPlannerNode()
  {
    if (pid_ <= 0) return;
    kill(pid_, SIGTERM);
    for (int i = 0; i < 100; ++i) {
      if (waitpid(pid_, nullptr, WNOHANG) == pid_) return;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    kill(pid_, SIGKILL);
    waitpid(pid_, nullptr, 0);
  }
  BehaviorPathPlannerNode(const BehaviorPathPlannerNode &) = delete;
  BehaviorPathPlannerNode & operator=(const BehaviorPathPlannerNode &) = delete;
private:
  pid_t pid_{-1};
};

class PlanningInterfaceTestManager {
public:
  PlanningInterfaceTestManager() : node_(std::make_shared<ros::NodeHandle>("/"))
  {
    output_ = node_->subscribe<autoware_internal_planning_msgs::PathWithLaneId>(
      "behavior_path_planner/output/path", 10,
      [this](const autoware_internal_planning_msgs::PathWithLaneIdConstPtr &) { ++received_; });
    timer_ = node_->createWallTimer(ros::WallDuration(0.05), [this](const ros::WallTimerEvent &) {
      for (const auto & entry : publishers_) {
        if (entry.first == "behavior_path_planner/input/vector_map" ||
            entry.first == "behavior_path_planner/input/route") continue;
        entry.second();
      }
    });
  }
  std::shared_ptr<ros::NodeHandle> getTestNode() { return node_; }
  size_t getReceivedTopicNum() const { return received_; }
  template <class T>
  void publishInput(const std::shared_ptr<BehaviorPathPlannerNode> &,
    const std::string & topic, const T & message)
  {
    // A replacement for an input updates the republished value, not a second
    // competing publisher that would keep replaying the previous fixture.
    const auto publisher = node_->advertise<T>(topic, 1, true);
    publishers_[topic] = [publisher, message] { publisher.publish(message); };
    publishers_.at(topic)();
    // Original publishInput defaults to three spin/sleep cycles of 100 ms.
    spinTestFor(0.3);
  }
private:
  std::shared_ptr<ros::NodeHandle> node_;
  ros::Subscriber output_;
  ros::WallTimer timer_;
  std::map<std::string, std::function<void()>> publishers_;
  size_t received_{0};
};

inline std::shared_ptr<PlanningInterfaceTestManager> generateTestManager()
{ return std::make_shared<PlanningInterfaceTestManager>(); }
inline std::shared_ptr<BehaviorPathPlannerNode> generateNode(
  const std::vector<std::string> & modules, const std::vector<std::string> & plugins)
{
  if (modules != std::vector<std::string>{"goal_planner"})
    throw std::runtime_error("This original-test fixture only loads Goal Planner configuration");
  return std::make_shared<BehaviorPathPlannerNode>(plugins);
}
inline void publishMandatoryTopics(const std::shared_ptr<PlanningInterfaceTestManager> & manager,
  const std::shared_ptr<BehaviorPathPlannerNode> & target)
{
  manager->publishInput(target, "behavior_path_planner/input/odometry", autoware::test_utils::makeInitialPose());
  manager->publishInput(target, "behavior_path_planner/input/accel", geometry_msgs::AccelWithCovarianceStamped{});
  manager->publishInput(target, "behavior_path_planner/input/perception", autoware_perception_msgs::PredictedObjects{});
  manager->publishInput(target, "behavior_path_planner/input/occupancy_grid_map", autoware::test_utils::makeCostMapMsg());
  manager->publishInput(target, "behavior_path_planner/input/scenario",
    autoware::test_utils::makeScenarioMsg(autoware_internal_planning_msgs::Scenario::LANEDRIVING));
  manager->publishInput(target, "behavior_path_planner/input/vector_map", autoware::test_utils::makeMapBinMsg());
  manager->publishInput(target, "behavior_path_planner/input/costmap", autoware::test_utils::makeCostMapMsg());
  manager->publishInput(target, "system/operation_mode/state", autoware_adapi_v1_msgs::OperationModeState{});
  manager->publishInput(target, "behavior_path_planner/input/lateral_offset", tier4_planning_msgs::LateralOffset{});
}
}
