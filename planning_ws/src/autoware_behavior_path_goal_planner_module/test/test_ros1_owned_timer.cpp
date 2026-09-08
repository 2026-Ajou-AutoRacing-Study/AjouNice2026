#include <autoware/behavior_path_goal_planner_module/ros1_owned_timer.hpp>
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <thread>

using autoware::behavior_path_planner::Ros1OwnedGoalTimer;
using namespace std::chrono_literals;

static bool waitFor(const std::function<bool()> & condition)
{
  const auto deadline = std::chrono::steady_clock::now() + 4s;
  while (!condition() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
  }
  return condition();
}

TEST(GoalOwnedTimer, StopJoinsRunningCallbackAndPreventsFutureCalls)
{
  ros::NodeHandle node;
  std::atomic<int> calls{0};
  std::atomic<bool> entered{false}, finished{false};
  Ros1OwnedGoalTimer timer(node, [&] {
    ++calls;
    entered = true;
    std::this_thread::sleep_for(400ms);
    finished = true;
  });
  timer.start();
  ASSERT_TRUE(waitFor([&] { return entered.load(); }));
  timer.stop();
  EXPECT_TRUE(finished);
  EXPECT_EQ(calls, 1);
  std::this_thread::sleep_for(1100ms);
  EXPECT_EQ(calls, 1);
}

TEST(GoalOwnedTimer, SeparateQueuesProgressWhileOtherCallbackRuns)
{
  ros::NodeHandle node;
  std::atomic<bool> entered{false}, second_during_first{false}, running{false};
  Ros1OwnedGoalTimer first(node, [&] {
    running = true;
    entered = true;
    std::this_thread::sleep_for(1600ms);
    running = false;
  });
  Ros1OwnedGoalTimer second(node, [&] { second_during_first = running.load(); });
  first.start();
  ASSERT_TRUE(waitFor([&] { return entered.load(); }));
  second.start();
  EXPECT_TRUE(waitFor([&] { return second_during_first.load(); }));
  second.stop();
  first.stop();
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "goal_owned_timer_test", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
