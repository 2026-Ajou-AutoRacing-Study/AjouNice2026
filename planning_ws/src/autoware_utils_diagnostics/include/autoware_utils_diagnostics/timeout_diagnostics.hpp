// Copyright 2025 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE_UTILS_DIAGNOSTICS__TIMEOUT_DIAGNOSTICS_HPP_
#define AUTOWARE_UTILS_DIAGNOSTICS__TIMEOUT_DIAGNOSTICS_HPP_

#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/time.h>

#include <functional>
#include <optional>
#include <string>

namespace autoware_utils_diagnostics
{
class TimeoutDiag : public diagnostic_updater::DiagnosticTask
{
public:
  struct Params
  {
    double warn_duration_;
    double error_duration_;
  };

  using DiagnosticStatus = diagnostic_msgs::DiagnosticStatus;
  using NowFunction = std::function<ros::Time()>;

  TimeoutDiag(const Params & params, NowFunction now, const std::string & name);
  TimeoutDiag(const Params & params, const std::string & name);
  void update();
  bool is_error() const { return level_ >= DiagnosticStatus::ERROR; }

private:
  void run(diagnostic_updater::DiagnosticStatusWrapper & stat) override;

  const Params params_;
  const NowFunction now_;
  std::optional<ros::Time> last_stamp_;
  uint8_t level_;
};
}  // namespace autoware_utils_diagnostics

#endif  // AUTOWARE_UTILS_DIAGNOSTICS__TIMEOUT_DIAGNOSTICS_HPP_
