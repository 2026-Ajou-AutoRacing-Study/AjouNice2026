#pragma once

namespace autoware::behavior_path_planner::utils::lane_change {
// Audit only: never approves, rejects, or mutates a candidate.
template<class Capture>
inline void audit_unapproved_candidate(
  bool enabled, bool normal, bool approved, bool native_safe, Capture capture) noexcept
{
  if (!enabled || !normal || approved || !native_safe) return;
  try { capture(); } catch (...) { /* Diagnostic failure cannot change native admission. */ }
}
}
