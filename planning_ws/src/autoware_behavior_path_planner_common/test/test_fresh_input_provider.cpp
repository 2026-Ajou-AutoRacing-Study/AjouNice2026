#include <autoware/behavior_path_planner_common/interface/fresh_input_provider.hpp>
#include <autoware/behavior_path_planner_common/planner_data.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <gtest/gtest.h>
#include <stdexcept>
#include <thread>

namespace fc = autoware::behavior_path_planner::fresh_commit;
using autoware::behavior_path_planner::PlannerData;
TEST(FreshInputProvider, MissingAndDisabledAreUnavailable)
{
  EXPECT_FALSE(fc::acquire());
  fc::ScopedProvider disabled({});
  EXPECT_FALSE(fc::acquire());
}
TEST(FreshInputProvider, NestedScopesRestoreAndStampCycle)
{
  fc::ScopedProvider outer([] { fc::Snapshot s; s.owner=reinterpret_cast<void*>(1); return s; });
  const auto first=fc::acquire(); ASSERT_TRUE(first);
  EXPECT_EQ(first->cycle,outer.cycle());
  {
    fc::ScopedProvider inner([] { fc::Snapshot s; s.owner=reinterpret_cast<void*>(2); return s; });
    EXPECT_EQ(fc::acquire()->owner,reinterpret_cast<void*>(2));
    EXPECT_NE(inner.cycle(),outer.cycle());
  }
  EXPECT_EQ(fc::acquire()->owner,first->owner);
  { fc::ScopedProvider disabled({}); EXPECT_FALSE(fc::acquire()); }
  EXPECT_EQ(fc::acquire()->cycle,first->cycle);
}
TEST(FreshInputProvider, ExceptionAndRecursionDoNotLeak)
{
  {
    fc::ScopedProvider throwing([]()->std::optional<fc::Snapshot>{throw std::runtime_error("test");});
    EXPECT_FALSE(fc::acquire()); EXPECT_FALSE(fc::acquire());
  }
  EXPECT_FALSE(fc::acquire());
  {
    fc::ScopedProvider recursive([] { EXPECT_FALSE(fc::acquire()); return fc::Snapshot{}; });
    EXPECT_TRUE(fc::acquire());
  }
  try { fc::ScopedProvider s([]{return fc::Snapshot{};}); throw 1; } catch (...) {}
  EXPECT_FALSE(fc::acquire());
}
TEST(FreshInputProvider, CannotEscapePlanningThread)
{
  fc::ScopedProvider scope([] {return fc::Snapshot{};});
  bool available=true;
  std::thread other([&]{available=fc::acquire().has_value();}); other.join();
  EXPECT_FALSE(available); EXPECT_TRUE(fc::acquire());
}
TEST(FreshInputProvider, RealMessagesPreserveIdentityAndOriginalState)
{
  auto baseline=std::make_shared<PlannerData>();
  nav_msgs::OdometryPtr old(new nav_msgs::Odometry),fresh(new nav_msgs::Odometry);
  old->header.stamp=ros::Time(10,1); fresh->header.stamp=ros::Time(11,2);
  baseline->self_odometry=old;
  fc::ScopedProvider scope([&] {
    auto copy=std::make_shared<PlannerData>(*baseline); copy->self_odometry=fresh;
    fc::Snapshot s; s.planner_data=copy; s.acquired_at=ros::Time(12,3); return s;
  });
  const auto result=fc::acquire(); ASSERT_TRUE(result);
  EXPECT_EQ(result->planner_data->self_odometry.get(),fresh.get());
  EXPECT_EQ(result->planner_data->self_odometry->header.stamp,ros::Time(11,2));
  EXPECT_EQ(baseline->self_odometry.get(),old.get());
  EXPECT_EQ(baseline->self_odometry->header.stamp,ros::Time(10,1));
}
TEST(FreshInputProvider, ConsumerMapIdentityUsesHandlerMapNotTransportMessage)
{
  auto map=std::make_shared<lanelet::LaneletMap>();
  auto handler=std::make_shared<autoware::route_handler::RouteHandler>(map);
  auto data=std::make_shared<PlannerData>(); data->route_handler=handler;
  const int transport_message=0;
  fc::ScopedProvider scope([&] {
    fc::Snapshot s; s.planner_data=data;
    s.route_handler=handler.get();
    s.map_identity=handler->getLaneletMapPtr().get();
    return s;
  });
  const auto snapshot=fc::acquire(); ASSERT_TRUE(snapshot);
  ASSERT_NE(snapshot->map_identity,nullptr);
  EXPECT_EQ(snapshot->route_handler,snapshot->planner_data->route_handler.get());
  EXPECT_EQ(snapshot->map_identity,
    snapshot->planner_data->route_handler->getLaneletMapPtr().get());
  EXPECT_NE(snapshot->map_identity,&transport_message);
  auto incorrect=*snapshot; incorrect.map_identity=&transport_message;
  EXPECT_NE(incorrect.map_identity,
    incorrect.planner_data->route_handler->getLaneletMapPtr().get());
}
