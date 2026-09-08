#include <autoware_adapi_v1_msgs/bounded_sequence_validator.hpp>

#include <gtest/gtest.h>

namespace msg = autoware_adapi_v1_msgs;

template<typename MessageT, typename Append>
void expect_cardinality_contract(
  MessageT message, Append append, const msg::BoundedFieldId expected_field)
{
  msg::BoundedSequenceViolation violation{};
  EXPECT_TRUE(msg::validate(message, &violation));
  append(message);
  EXPECT_TRUE(msg::validate(message, &violation));
  append(message);
  EXPECT_FALSE(msg::validate(message, &violation));
  EXPECT_EQ(expected_field, violation.field);
  EXPECT_EQ(2U, violation.observed);
  EXPECT_EQ(1U, msg::BoundedSequenceViolation::maximum);
}

TEST(BoundedSequenceValidator, RouteDataZeroOneTwo)
{
  expect_cardinality_contract(msg::Route{},
    [](msg::Route & value) { value.data.emplace_back(); }, msg::BoundedFieldId::kRouteData);
}

TEST(BoundedSequenceValidator, SteeringFactorCooperationZeroOneTwo)
{
  expect_cardinality_contract(msg::SteeringFactor{},
    [](msg::SteeringFactor & value) { value.cooperation.emplace_back(); },
    msg::BoundedFieldId::kSteeringFactorCooperation);
}

TEST(BoundedSequenceValidator, VelocityFactorCooperationZeroOneTwo)
{
  expect_cardinality_contract(msg::VelocityFactor{},
    [](msg::VelocityFactor & value) { value.cooperation.emplace_back(); },
    msg::BoundedFieldId::kVelocityFactorCooperation);
}

TEST(BoundedSequenceValidator, InitializeLocalizationPoseZeroOneTwo)
{
  expect_cardinality_contract(msg::InitializeLocalizationRequest{},
    [](msg::InitializeLocalizationRequest & value) { value.pose.emplace_back(); },
    msg::BoundedFieldId::kInitializeLocalizationRequestPose);
}
