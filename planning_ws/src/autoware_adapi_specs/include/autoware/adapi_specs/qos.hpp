// Copyright 2026
//
// ROS1 delivery-policy representation for the AD API specification headers.
// This is deliberately metadata, not a replacement transport implementation.

#ifndef AUTOWARE__ADAPI_SPECS__QOS_HPP_
#define AUTOWARE__ADAPI_SPECS__QOS_HPP_

#include <cstddef>
#include <cstdint>

// Keep the public names used by the ROS2 specification headers so a ported
// consumer can retain the declared contract without depending on rclcpp/rmw.
enum class autoware_adapi_reliability_policy_t : std::uint8_t
{
  reliable,
  best_effort,
};

enum class autoware_adapi_durability_policy_t : std::uint8_t
{
  volatile_data,
  transient_local,
};

constexpr autoware_adapi_reliability_policy_t RMW_QOS_POLICY_RELIABILITY_RELIABLE =
  autoware_adapi_reliability_policy_t::reliable;
constexpr autoware_adapi_reliability_policy_t RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT =
  autoware_adapi_reliability_policy_t::best_effort;
constexpr autoware_adapi_durability_policy_t RMW_QOS_POLICY_DURABILITY_VOLATILE =
  autoware_adapi_durability_policy_t::volatile_data;
constexpr autoware_adapi_durability_policy_t RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL =
  autoware_adapi_durability_policy_t::transient_local;

namespace autoware::adapi_specs
{

struct Ros1DeliveryPolicy
{
  std::size_t queue_size;
  bool latch;
  autoware_adapi_reliability_policy_t requested_reliability;
  // TCPROS/UDPROS selection does not prove DDS reliable/best-effort semantics.
  bool reliability_exactly_representable;
};

template<typename Spec>
constexpr Ros1DeliveryPolicy ros1_delivery_policy()
{
  return {
    Spec::depth,
    Spec::durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    Spec::reliability,
    false,
  };
}

}  // namespace autoware::adapi_specs

#endif  // AUTOWARE__ADAPI_SPECS__QOS_HPP_
