#include "autoware/behavior_path_lane_change_module/utils/bounded_output_policy.hpp"
#include "autoware/behavior_path_lane_change_module/utils/selected_output_prediction.hpp"
#include <gtest/gtest.h>
namespace lc=autoware::behavior_path_planner::utils::lane_change;
namespace {
lc::SafetyPath selected_path(bool stop) {
  lc::SafetyPath path;
  for(size_t i=0;i<=30;++i) {
    decltype(path.points)::value_type point;
    point.point.pose.position.x=i;
    point.point.pose.position.z=28.0+0.1*i;
    point.point.pose.orientation.w=1;
    point.point.longitudinal_velocity_mps=stop ? std::sqrt(std::max(0.0,16.0-2.0*i)) :
      std::sqrt(16.0+2.0*i);
    path.points.push_back(point);
  }
  return path;
}
}
TEST(BoundedOutputPolicy, SelectedStoppedOutputHoldsPoseAndPreparePhase) {
  const auto path=selected_path(true);
  const auto predictions=lc::selected_output_predictions(path,0,20,12,4,-2,30,.2,3,false,true);
  ASSERT_EQ(predictions.size(),1U);
  EXPECT_GT(predictions.front().prepare_time,3.0); // never reaches LC start
  EXPECT_NEAR(predictions.front().prediction.back().pose.position.x,7.5,0.15);
  EXPECT_LT(predictions.front().prediction.back().velocity,2.0);
  // A longer horizon reaches the genuine stop, then remains stationary.
  const auto stopped=lc::selected_output_predictions(path,0,20,12,4,-2,30,.2,5,false,true);
  EXPECT_NEAR(stopped.front().prediction.back().pose.position.x,8.0,1e-9);
  EXPECT_EQ(stopped.front().prediction.back().velocity,0);
  EXPECT_GT(stopped.front().prepare_time,5.0);
}
TEST(BoundedOutputPolicy, SelectedAccelerationUsesPlanarArcAndRemainingPrepare) {
  const auto path=selected_path(false);
  const auto predictions=lc::selected_output_predictions(path,0,20,5,4,1,30,.1,3,false,true);
  ASSERT_EQ(predictions.size(),1U);
  // ROS path velocities are float32, unlike the analytic double reference.
  EXPECT_NEAR(predictions.front().prediction.back().pose.position.x,16.5,1e-5);
  EXPECT_NEAR(predictions.front().prediction.back().velocity,7.0,1e-5);
  EXPECT_NEAR(predictions.front().prepare_time,std::sqrt(26.0)-4.0,.002);
  const auto crossed=lc::selected_output_predictions(path,6,20,5,
    path.points[6].point.longitudinal_velocity_mps,1,30,.1,2.5,false,true);
  EXPECT_EQ(crossed.front().prepare_time,0);
  const auto both=lc::selected_output_predictions(path,0,20,5,4,0,30,.1,3,false,false);
  EXPECT_EQ(both.size(),2U);
}
TEST(BoundedOutputPolicy, SelectedOutputRequiresExactSameDecisionInputs) {
  lc::SelectedOutputIdentity selected;
  selected.cycle=4;selected.odom_stamp=100;selected.object_stamp=101;
  selected.route_stamp=1;selected.acceleration_stamp=99;
  selected.odom=1;selected.objects=2;selected.acceleration=3;selected.route=4;
  selected.route_uuid="route-a";selected.measured_acceleration=1.2;
  selected.diagnostic_only=true;
  EXPECT_TRUE(selected.matches(selected));
  auto changed=selected; ++changed.cycle; EXPECT_FALSE(selected.matches(changed));
  changed=selected; ++changed.odom_stamp; EXPECT_FALSE(selected.matches(changed));
  changed=selected; ++changed.object_stamp; EXPECT_FALSE(selected.matches(changed));
  changed=selected; ++changed.route_stamp; EXPECT_FALSE(selected.matches(changed));
  changed=selected; ++changed.acceleration_stamp; EXPECT_FALSE(selected.matches(changed));
  changed=selected; ++changed.objects; EXPECT_FALSE(selected.matches(changed));
  changed=selected; changed.route_uuid="route-b"; EXPECT_FALSE(selected.matches(changed));
  changed=selected; changed.measured_acceleration=1.3; EXPECT_FALSE(selected.matches(changed));
  changed=selected; changed.diagnostic_only=false; EXPECT_FALSE(selected.matches(changed));
}
TEST(BoundedOutputPolicy, MissingOrInvalidIdentityCannotReuseSelectedSafety) {
  lc::SelectedOutputIdentity missing;
  EXPECT_FALSE(missing.matches(missing));
  missing.odom_stamp=1;missing.object_stamp=2;
  missing.measured_acceleration=std::nan("");
  EXPECT_FALSE(missing.matches(missing));
}
TEST(BoundedOutputPolicy, DefaultPreservesOriginalAcceptance) {
  EXPECT_FALSE(lc::owns_bounded_longitudinal_output(false,true,false,false));
  EXPECT_TRUE(lc::may_publish_original_longitudinal(true,false));
  EXPECT_FALSE(lc::may_publish_original_longitudinal(false,false));
}
TEST(BoundedOutputPolicy, AlternatingOriginalSafetyNeverBypassesBoundedOwner) {
  const bool owns=lc::owns_bounded_longitudinal_output(true,true,false,false);
  EXPECT_TRUE(owns);
  for(bool safe:{false,true,false,true,true})
    EXPECT_FALSE(lc::may_publish_original_longitudinal(safe,owns));
}
TEST(BoundedOutputPolicy, AbortAndStopKeepExistingDispatch) {
  EXPECT_FALSE(lc::owns_bounded_longitudinal_output(true,true,true,false));
  EXPECT_FALSE(lc::owns_bounded_longitudinal_output(true,true,false,true));
  EXPECT_FALSE(lc::owns_bounded_longitudinal_output(true,false,false,false));
  // No persisted state: a fresh normal execution gets an owner, a reset has none.
  EXPECT_TRUE(lc::owns_bounded_longitudinal_output(true,true,false,false));
}
TEST(BoundedOutputPolicy, LegacyCancelsBeforeGridAndNeverTwice) {
  lc::ValidatedCancelOrder order;
  EXPECT_TRUE(order.claim(false,false,false));
  EXPECT_FALSE(order.claim(false,false,true));
  EXPECT_FALSE(order.claim(false,false,false));
}
TEST(BoundedOutputPolicy, BoundedOwnerSearchesGridBeforeCancelEvenIfNativeUnsafe) {
  lc::ValidatedCancelOrder order;
  EXPECT_FALSE(order.claim(true,false,false));
  EXPECT_FALSE(order.attempted);
  // A safe grid would return here without claiming or mutating cancellation.
  EXPECT_TRUE(order.claim(true,false,true));
  EXPECT_TRUE(order.attempted);
  EXPECT_FALSE(order.claim(true,false,true)); // exception/fallback cannot retry
}
TEST(BoundedOutputPolicy, NativeSafeDoesNotCauseUnnecessaryCancellation) {
  lc::ValidatedCancelOrder order;
  EXPECT_FALSE(order.claim(true,true,false));
  EXPECT_FALSE(order.claim(true,true,true));
  EXPECT_FALSE(order.attempted);
}
