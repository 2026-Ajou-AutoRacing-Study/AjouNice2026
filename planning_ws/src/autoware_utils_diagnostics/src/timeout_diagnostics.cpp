// Copyright 2025 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.

#include "autoware_utils_diagnostics/timeout_diagnostics.hpp"

namespace autoware_utils_diagnostics
{
TimeoutDiag::TimeoutDiag(const Params & params, NowFunction now, const std::string & name)
: DiagnosticTask(name), params_(params), now_(std::move(now)), level_(DiagnosticStatus::STALE)
{
}

TimeoutDiag::TimeoutDiag(const Params & params, const std::string & name)
: TimeoutDiag(params, []() { return ros::Time::now(); }, name)
{
}

void TimeoutDiag::update()
{
  last_stamp_ = now_();
}

void TimeoutDiag::run(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!last_stamp_) {
    stat.summary(DiagnosticStatus::ERROR, "no data");
    return;
  }
  const auto duration = (now_() - *last_stamp_).toSec();
  if (duration < params_.warn_duration_) {
    level_ = DiagnosticStatus::OK;
  } else if (duration < params_.error_duration_) {
    level_ = DiagnosticStatus::WARN;
  } else {
    level_ = DiagnosticStatus::ERROR;
  }
  stat.summary(level_, "");
  stat.add("duration", duration);
}
}  // namespace autoware_utils_diagnostics
