#ifndef AUTOWARE_ADAPI_V1_MSGS__BOUNDED_SEQUENCE_VALIDATOR_HPP_
#define AUTOWARE_ADAPI_V1_MSGS__BOUNDED_SEQUENCE_VALIDATOR_HPP_

#include <autoware_adapi_v1_msgs/InitializeLocalization.h>
#include <autoware_adapi_v1_msgs/Route.h>
#include <autoware_adapi_v1_msgs/SteeringFactor.h>
#include <autoware_adapi_v1_msgs/VelocityFactor.h>

#include <cstddef>

namespace autoware_adapi_v1_msgs
{
enum class BoundedFieldId {
  kRouteData,
  kSteeringFactorCooperation,
  kVelocityFactorCooperation,
  kInitializeLocalizationRequestPose,
};

struct BoundedSequenceViolation {
  BoundedFieldId field;
  std::size_t observed;
  enum : std::size_t { maximum = 1 };
};

inline bool validate_size(
  const BoundedFieldId field, const std::size_t observed, BoundedSequenceViolation * violation)
{
  if (observed <= BoundedSequenceViolation::maximum) {
    return true;
  }
  if (violation != nullptr) {
    *violation = {field, observed};
  }
  return false;
}

inline bool validate(const Route & message, BoundedSequenceViolation * violation = nullptr)
{
  return validate_size(BoundedFieldId::kRouteData, message.data.size(), violation);
}

inline bool validate(const SteeringFactor & message, BoundedSequenceViolation * violation = nullptr)
{
  return validate_size(
    BoundedFieldId::kSteeringFactorCooperation, message.cooperation.size(), violation);
}

inline bool validate(const VelocityFactor & message, BoundedSequenceViolation * violation = nullptr)
{
  return validate_size(
    BoundedFieldId::kVelocityFactorCooperation, message.cooperation.size(), violation);
}

inline bool validate(
  const InitializeLocalizationRequest & request, BoundedSequenceViolation * violation = nullptr)
{
  return validate_size(
    BoundedFieldId::kInitializeLocalizationRequestPose, request.pose.size(), violation);
}
}  // namespace autoware_adapi_v1_msgs

#endif  // AUTOWARE_ADAPI_V1_MSGS__BOUNDED_SEQUENCE_VALIDATOR_HPP_
