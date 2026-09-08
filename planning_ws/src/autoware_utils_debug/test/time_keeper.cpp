#include <autoware_utils_debug/time_keeper.hpp>

#include <gtest/gtest.h>

#include <ros/console.h>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <array>
#include <cstdio>
#include <unistd.h>

namespace
{
std::string run_logger_probe(const char * argument)
{
  std::array<char, 4096> executable{};
  const auto count = readlink("/proc/self/exe", executable.data(), executable.size() - 1);
  if (count <= 0) return {};
  const std::string path(executable.data(), static_cast<std::size_t>(count));
  const auto slash = path.rfind('/');
  if (slash == std::string::npos) return {};
  const std::string command =
    "ROSCONSOLE_FORMAT='${logger}|${severity}|${message}\\n' '" +
    path.substr(0, slash + 1) + "time_keeper_logger_probe' " + argument + " 2>&1";
  std::string output;
  FILE * pipe = popen(command.c_str(), "r");
  if (!pipe) return output;
  std::array<char, 256> buffer{};
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe)) output += buffer.data();
  pclose(pipe);
  return output;
}

TEST(TimeKeeperActualRender, ReportsNestedTree)
{
  std::ostringstream stream;
  autoware_utils_debug::TimeKeeper keeper(&stream);
  keeper.start_track("root");
  keeper.comment("outer");
  keeper.start_track("child");
  keeper.end_track("child");
  keeper.end_track("root");
  EXPECT_NE(stream.str().find("root"), std::string::npos);
  EXPECT_NE(stream.str().find("child"), std::string::npos);
}

TEST(TimeKeeperActualRender, PreservesOrderFailure)
{
  autoware_utils_debug::TimeKeeper keeper;
  keeper.start_track("root");
  EXPECT_THROW(keeper.end_track("wrong"), std::runtime_error);
  keeper.end_track("root");
}

TEST(TimeKeeperActualRender, CrossThreadWarningUsesTimeKeeperLoggerAndExactUpstreamMessage)
{
  const std::string control = run_logger_probe("");
  ASSERT_NE(control.find("ros.TimeKeeper|WARN|timekeeper-capture-control"), std::string::npos) << control;
  const std::string expected =
    "TimeKeeper::start_track(cross_thread) is called from a different thread. Ignoring the call.";
  const std::string captured = run_logger_probe("cross_thread");
  EXPECT_NE(captured.find("ros.TimeKeeper|WARN|" + expected), std::string::npos) << captured;
}

TEST(TimeKeeperActualRender, NamedLoggerMutationNegativeRejectsUnnamedOrShortenedAlternatives)
{
  const auto source = std::string(__FILE__).substr(0, std::string(__FILE__).rfind("/test/")) + "/src/time_keeper.cpp";
  std::ifstream file(source);
  const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  ASSERT_FALSE(text.empty());
  EXPECT_NE(text.find("ROS_LOG(::ros::console::levels::Warn, \"ros.TimeKeeper\", \"%s\", warning.c_str());"), std::string::npos);
  EXPECT_EQ(text.find("ROS_LOG(::ros::console::levels::Warn, \"TimeKeeper\", \"%s\", warning.c_str());"), std::string::npos);
  EXPECT_NE(text.find("is called from a different thread. Ignoring the call."), std::string::npos);
  EXPECT_EQ(text.find("ROS_WARN_STREAM"), std::string::npos);
}
}
