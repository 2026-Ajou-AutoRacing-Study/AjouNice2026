#pragma once

// ROS1_NO_AGNOCAST_FALLBACK: this header intentionally exposes only native
// ROS1 message_filters types.  It has no IPC, allocator, component, executor,
// or runtime transport-switching surface.
#if defined(USE_AGNOCAST_ENABLED)
#error "USE_AGNOCAST_ENABLED is unsupported by the ROS1 disabled profile; configure with ENABLE_AGNOCAST=0 or unset it."
#endif

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

namespace autoware
{
namespace agnocast_wrapper
{
namespace message_filters
{

template <class M>
using Subscriber = ::message_filters::Subscriber<M>;

template <typename M0, typename M1>
using ApproximateTimeSynchronizer =
  ::message_filters::Synchronizer<::message_filters::sync_policies::ApproximateTime<M0, M1>>;

namespace sync_policies
{
template <typename M0, typename M1>
using ApproximateTime = ::message_filters::sync_policies::ApproximateTime<M0, M1>;
}  // namespace sync_policies

template <typename Policy>
using Synchronizer = ::message_filters::Synchronizer<Policy>;

}  // namespace message_filters
}  // namespace agnocast_wrapper
}  // namespace autoware

