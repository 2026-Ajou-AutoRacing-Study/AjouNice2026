#include <autoware_utils_tf/self_pose_listener.hpp>
#include <autoware_utils_tf/transform_listener.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <atomic>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

namespace
{
bool wait_for(const std::atomic<bool> & done, const double seconds)
{
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(seconds);
  while (!done.load() && ros::WallTime::now() < deadline) { ros::WallDuration(0.02).sleep(); }
  return done.load();
}
geometry_msgs::TransformStamped transform(const ros::Time & stamp)
{
  geometry_msgs::TransformStamped value;
  value.header.stamp = stamp; value.header.frame_id = "map"; value.child_frame_id = "base_link";
  value.transform.translation.x = 12.5; value.transform.translation.y = -3.0; value.transform.translation.z = 0.25;
  value.transform.rotation.w = 1.0; return value;
}
bool success(const ros::NodeHandle & nh)
{
  ros::param::set("/use_sim_time", false);
  autoware_utils_tf::TransformListener listener(nh);
  tf2_ros::TransformBroadcaster broadcaster;
  const geometry_msgs::TransformStamped known = transform(ros::Time::now());
  std::thread delayed_publisher([&broadcaster, &known]() {
    ros::WallDuration(0.25).sleep();
    for (int i = 0; i < 15; ++i) { geometry_msgs::TransformStamped sent = known; sent.header.stamp = ros::Time::now(); broadcaster.sendTransform(sent); ros::WallDuration(0.03).sleep(); }
  });
  const auto waited = listener.get_transform("map", "base_link", ros::Time(0), ros::Duration(2.0));
  delayed_publisher.join();
  const auto latest = listener.get_latest_transform("map", "base_link");
  const auto timed = listener.get_transform("map", "base_link", latest ? latest->header.stamp : ros::Time(0), ros::Duration(0.5));
  const auto missing = listener.get_latest_transform("missing_frame", "base_link");
  if (!waited || !latest || !timed || missing) { std::cerr << "lookup/null oracle failed" << std::endl; return false; }
  if (std::fabs(latest->transform.translation.x - known.transform.translation.x) > 1e-9 || std::fabs(timed->transform.translation.y - known.transform.translation.y) > 1e-9) { std::cerr << "transform value mismatch" << std::endl; return false; }
  autoware_utils_tf::SelfPoseListener pose_listener(nh);
  std::atomic<bool> pose_done(false); std::atomic<bool> pose_sent(false);
  const ros::WallTime pose_start = ros::WallTime::now();
  std::thread pose_waiter([&pose_listener, &pose_done]() { pose_listener.wait_for_first_pose(); pose_done.store(true); });
  tf2_ros::TransformBroadcaster pose_broadcaster;
  std::thread pose_publisher([&pose_broadcaster, &known, &pose_sent]() {
    ros::WallDuration(0.25).sleep();
    for (int i = 0; i < 10; ++i) { geometry_msgs::TransformStamped sent = known; sent.header.stamp = ros::Time::now(); pose_broadcaster.sendTransform(sent); pose_sent.store(true); ros::WallDuration(0.03).sleep(); }
  });
  const bool got_pose = wait_for(pose_done, 7.0);
  const double pose_elapsed = (ros::WallTime::now() - pose_start).toSec();
  pose_publisher.join();
  if (!got_pose) { ros::shutdown(); }
  pose_waiter.join();
  // The transform is sent after the first immediate poll and observed only after the next 0.2 Hz WallRate poll.
  if (!got_pose || !pose_sent.load() || pose_elapsed < 4.5 || pose_elapsed > 6.5) {
    std::cerr << "self pose wall-poll delivery bound failed elapsed=" << pose_elapsed << std::endl; return false;
  }
  return true;
}
bool timeout_mode(const ros::NodeHandle & nh)
{
  ros::param::set("/use_sim_time", false);
  autoware_utils_tf::TransformListener listener(nh);
  const ros::WallTime start = ros::WallTime::now();
  const auto absent = listener.get_transform("missing_frame", "base_link", ros::Time(0), ros::Duration(0.4));
  const double elapsed = (ros::WallTime::now() - start).toSec();
  if (absent || elapsed < 0.3 || elapsed > 1.5) {
    std::cerr << "listener timeout safety bound failed elapsed=" << elapsed << std::endl; return false;
  }
  return true;
}
bool shutdown_mode(const ros::NodeHandle & nh)
{
  ros::param::set("/use_sim_time", false); autoware_utils_tf::SelfPoseListener pose_listener(nh); std::atomic<bool> pose_done(false);
  std::thread pose_waiter([&pose_listener, &pose_done]() { pose_listener.wait_for_first_pose(); pose_done.store(true); });
  ros::WallDuration(0.2).sleep(); ros::shutdown(); const bool stopped = wait_for(pose_done, 6.5); pose_waiter.join();
  if (!stopped) { std::cerr << "self pose did not exit after shutdown" << std::endl; } return stopped;
}
bool paused_clock_mode(const ros::NodeHandle & nh)
{
  ros::param::set("/use_sim_time", true); autoware_utils_tf::SelfPoseListener pose_listener(nh); std::atomic<bool> pose_done(false);
  std::thread pose_waiter([&pose_listener, &pose_done]() { pose_listener.wait_for_first_pose(); pose_done.store(true); });
  ros::WallDuration(1.0).sleep(); tf2_ros::StaticTransformBroadcaster broadcaster; broadcaster.sendTransform(transform(ros::Time(0)));
  const bool completed = wait_for(pose_done, 6.0); if (!completed) { ros::shutdown(); } pose_waiter.join();
  if (!completed) { std::cerr << "paused /clock watchdog expired" << std::endl; } return completed;
}
}  // namespace
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "autoware_utils_tf_runtime_oracle"); if (!ros::master::check()) { std::cerr << "ROS master unavailable" << std::endl; return 2; }
  ros::NodeHandle nh; const std::string mode = argc == 2 ? argv[1] : "";
  const bool passed = mode == "success" ? success(nh) : mode == "timeout" ? timeout_mode(nh) : mode == "shutdown" ? shutdown_mode(nh) : mode == "paused-clock" ? paused_clock_mode(nh) : false;
  if (mode != "success" && mode != "timeout" && mode != "shutdown" && mode != "paused-clock") { std::cerr << "mode must be success, timeout, shutdown, or paused-clock" << std::endl; return 2; }
  if (!passed) { return 1; } std::cout << "PASS " << mode << std::endl; return 0;
}
