#include <autoware/behavior_path_planner_common/interface/fresh_input_provider.hpp>
#include <atomic>
#include <utility>

namespace autoware::behavior_path_planner::fresh_commit
{
namespace
{
thread_local ScopedProvider * active = nullptr;
std::atomic<uint64_t> next_cycle{0};
}
ScopedProvider::ScopedProvider(Provider provider)
: provider_(std::move(provider)), previous_(active), cycle_(++next_cycle)
{
  active = this;
}
ScopedProvider::~ScopedProvider() { active = previous_; }
std::optional<Snapshot> acquire()
{
  auto * scope = active;
  if (!scope || !scope->provider_ || scope->invoking_) return std::nullopt;
  scope->invoking_ = true;
  try {
    auto result = scope->provider_();
    scope->invoking_ = false;
    if (result) result->cycle = scope->cycle_;
    return result;
  } catch (...) {
    scope->invoking_ = false;
    return std::nullopt;
  }
}
}  // namespace autoware::behavior_path_planner::fresh_commit
