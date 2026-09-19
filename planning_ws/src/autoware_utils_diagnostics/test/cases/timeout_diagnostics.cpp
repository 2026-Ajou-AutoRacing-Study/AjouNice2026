#include "autoware_utils_diagnostics/timeout_diagnostics.hpp"

#include <gtest/gtest.h>

namespace
{
diagnostic_updater::DiagnosticStatusWrapper run_task(
  autoware_utils_diagnostics::TimeoutDiag & timeout_diag)
{
  diagnostic_updater::DiagnosticStatusWrapper status;
  diagnostic_updater::DiagnosticTask & task = timeout_diag;
  task.run(status);
  return status;
}
}  // namespace

TEST(TimeoutDiagTest, Instantiation)
{
  autoware_utils_diagnostics::TimeoutDiag::Params params{1.0, 2.0};
  autoware_utils_diagnostics::TimeoutDiag timeout_diag(params, "test_diag");
}

TEST(TimeoutDiagTest, StrictThresholdsAndNoData)
{
  ros::Time current(100, 0);
  autoware_utils_diagnostics::TimeoutDiag::Params params{1.0, 2.0};
  autoware_utils_diagnostics::TimeoutDiag timeout_diag(
    params, [&current]() { return current; }, "test_diag");

  auto status = run_task(timeout_diag);
  EXPECT_EQ(status.level, diagnostic_msgs::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "no data");
  EXPECT_TRUE(timeout_diag.is_error());

  timeout_diag.update();
  current = ros::Time(100, 999999999);
  status = run_task(timeout_diag);
  EXPECT_EQ(status.level, diagnostic_msgs::DiagnosticStatus::OK);
  EXPECT_FALSE(timeout_diag.is_error());

  current = ros::Time(101, 0);
  status = run_task(timeout_diag);
  EXPECT_EQ(status.level, diagnostic_msgs::DiagnosticStatus::WARN);
  EXPECT_FALSE(timeout_diag.is_error());

  current = ros::Time(101, 999999999);
  status = run_task(timeout_diag);
  EXPECT_EQ(status.level, diagnostic_msgs::DiagnosticStatus::WARN);

  current = ros::Time(102, 0);
  status = run_task(timeout_diag);
  EXPECT_EQ(status.level, diagnostic_msgs::DiagnosticStatus::ERROR);
  EXPECT_TRUE(timeout_diag.is_error());
}
