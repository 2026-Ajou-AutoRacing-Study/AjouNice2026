#include <autoware/behavior_path_bidirectional_traffic_module/bidirectional_lanelets.hpp>
#include <autoware/behavior_path_bidirectional_traffic_module/oncoming_car.hpp>
#include <autoware/route_handler/route_handler.hpp>

#include <autoware_perception_msgs/ObjectClassification.h>
#include <autoware_perception_msgs/PredictedObject.h>
#include <autoware_perception_msgs/PredictedObjects.h>

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <gtest/gtest.h>

TEST(Aw149BidirectionalOncoming, MapLaneAndObjectPromotion)
{
  lanelet::ErrorMessages errors;
  lanelet::projection::UtmProjector projector(lanelet::Origin({35.228457, 138.803539}));
  auto loaded_map = lanelet::load(AW149_BIDIRECTIONAL_TEST_MAP, projector, &errors);
  ASSERT_TRUE(loaded_map) << (errors.empty() ? "failed to load map" : errors.front());
  std::shared_ptr<lanelet::LaneletMap> mutable_map(std::move(loaded_map));
  lanelet::LaneletMapConstPtr map = mutable_map;
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map);
  const auto lanes = autoware::behavior_path_planner::ConnectedBidirectionalLanelets::
    search_bidirectional_lanes_on_map(
      *map,
      [&route_handler](const lanelet::ConstLanelet & lane) { return route_handler->getNextLanelets(lane); },
      [&route_handler](const lanelet::ConstLanelet & lane) { return route_handler->getPreviousLanelets(lane); });
  ASSERT_EQ(lanes.size(), 8U);
  const auto lane = lanes.front();
  geometry_msgs::Pose ego = lane->get_center_line().compute(0.0);
  const auto current = autoware::behavior_path_planner::get_bidirectional_lanelets_where_ego_is(
    ego, {3.8, 1.1, 1.9}, lanes);
  ASSERT_TRUE(current.has_value());
  autoware::behavior_path_planner::OncomingCars cars(*current, 0.1);
  autoware_perception_msgs::PredictedObject object;
  object.object_id.uuid[0] = 0x42;
  object.kinematics.initial_pose_with_covariance.pose = lane->get_opposite()->get_center_line().compute(1.0);
  object.kinematics.initial_twist_with_covariance.twist.linear.x = 3.0;
  autoware_perception_msgs::ObjectClassification classification;
  classification.label = autoware_perception_msgs::ObjectClassification::CAR;
  object.classification.push_back(classification);
  autoware_perception_msgs::PredictedObjects objects;
  objects.objects.push_back(object);
  cars.update(objects, ego, ros::Time(0));
  cars.update(objects, ego, ros::Time(1));
  ASSERT_TRUE(cars.get_front_oncoming_car().has_value());
  EXPECT_EQ(cars.get_front_oncoming_car()->get_uuid().uuid[0], 0x42);
}
