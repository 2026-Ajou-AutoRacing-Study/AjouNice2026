#include "autoware/behavior_path_lane_change_module/utils/measured_motion_policy.hpp"
#include <gtest/gtest.h>
#include <limits>
using autoware::behavior_path_planner::utils::lane_change::measured_failure_is_diagnostic;
TEST(MeasuredMotionPolicy, OffAndPlannedAlwaysHard) {
  for(double t:{0.,.5,3.}) {
    EXPECT_FALSE(measured_failure_is_diagnostic(false,1,t));
    EXPECT_FALSE(measured_failure_is_diagnostic(true,0,t));
  }
}
TEST(MeasuredMotionPolicy, PresentAndInvalidAlwaysHard) {
  for(double t:{-1.,0.,1e-6,std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
    EXPECT_FALSE(measured_failure_is_diagnostic(true,1,t));
  EXPECT_TRUE(measured_failure_is_diagnostic(true,1,.5));
}
TEST(MeasuredMotionPolicy, R25DecisionTruthTable) {
  // Full-core replay facts: r1 planned unsafe; r2/r3 planned safe, all measured unsafe.
  for(bool planned_safe:{false,true,true}) {
    const bool off_accept=planned_safe && measured_failure_is_diagnostic(false,1,2.5);
    const bool on_accept=planned_safe && measured_failure_is_diagnostic(true,1,2.5);
    EXPECT_FALSE(off_accept);
    EXPECT_EQ(on_accept,planned_safe);
  }
}
