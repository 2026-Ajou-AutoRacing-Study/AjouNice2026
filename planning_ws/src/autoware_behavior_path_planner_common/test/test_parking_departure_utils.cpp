#include <autoware/behavior_path_planner_common/utils/parking_departure/utils.hpp>

#include <gtest/gtest.h>

namespace parking = autoware::behavior_path_planner::utils::parking_departure;
namespace safety = autoware::behavior_path_planner::utils::path_safety_checker;

TEST(ParkingDepartureSourceAdapter, PreservesPathPropertyAndCommonData)
{
  auto parameters = std::make_shared<safety::EgoPredictedPathParams>();
  parameters->min_acceleration = 1.0;
  parking::updatePathProperty(parameters, {3.0, 0.2});
  EXPECT_DOUBLE_EQ(parameters->max_velocity, 3.0);
  EXPECT_DOUBLE_EQ(parameters->acceleration, 1.0);

  autoware::behavior_path_planner::StartGoalPlannerData data;
  boost::array<uint8_t, 16> id{};
  id.at(0) = 1;
  data.collision_check[id] = {};
  parking::initializeCollisionCheckDebugMap(data.collision_check);
  EXPECT_TRUE(data.collision_check.empty());
  std::vector<safety::PoseWithVelocityStamped> path(1);
  path.front().velocity = 2.0;
  parking::updateSafetyCheckTargetObjectsData(data, {}, {}, path);
  ASSERT_EQ(data.ego_predicted_path.size(), 1U);
  EXPECT_DOUBLE_EQ(data.ego_predicted_path.front().velocity, 2.0);
}
