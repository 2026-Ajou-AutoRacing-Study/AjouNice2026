#include <autoware_utils_debug/time_keeper.hpp>

#include <ros/ros.h>

#include <string>
#include <thread>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "time_keeper_logger_probe", ros::init_options::AnonymousName);
  if (argc == 2 && std::string(argv[1]) == "cross_thread") {
    autoware_utils_debug::TimeKeeper keeper;
    keeper.start_track("root");
    std::thread other([&keeper] { keeper.start_track("cross_thread"); });
    other.join();
    keeper.end_track("root");
  } else {
    ROS_LOG(::ros::console::levels::Warn, "ros.TimeKeeper", "%s", "timekeeper-capture-control");
  }
  ros::shutdown();
  return 0;
}
