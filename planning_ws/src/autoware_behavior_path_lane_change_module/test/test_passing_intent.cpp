#include <autoware/behavior_path_lane_change_module/utils/passing_intent.hpp>
#include <gtest/gtest.h>
using namespace autoware::behavior_path_planner::passing_intent;
struct PassingIntentTest : testing::Test {
  Intent i;
  Context c;
  void SetUp() override {
    i.id="intent";i.outbound_uuid="owner";i.lead_uuid="lead";
    i.route=route_uuid;i.boot="boot";i.map=map_sha;i.direction="right";i.phase="PASSING";
    i.route_stamp=1;i.stamp=10;i.ttl=.5;i.source={1,2};i.target={3,4};
    c.enabled=c.route_verified=c.corridor_verified=c.normal=c.distances_valid=true;
    c.near_terminal=false;c.route=route_uuid;c.boot="boot";c.direction="left";
    c.now=10.1;c.route_stamp=1;c.ownership_stamp=10;c.intent_wall_age=.1;
    c.remaining=500;c.terminal_distance=500;c.lateral_distance=500;c.return_buffer=60;c.speed=20;
    c.acceleration=0;c.deceleration=3;c.jerk=10;c.front_margin=4;
    c.current_lane=3;c.target_lane=1;c.corridor={1,2,3,4};
  }
};
TEST_F(PassingIntentTest, RegisteredWaitingCandidateAcknowledgesLateIntentOnUpdate) {
  // No new execution-request call: the same registered external candidate is
  // updated first without an intent, then with an ARM intent arriving later.
  c.normal=false;i.phase="ARM";
  Intent absent;
  ASSERT_TRUE(refresh_waiting_candidate_ack(true,true,true,false,false));
  EXPECT_FALSE(decide(absent,c,false).acknowledged);
  const auto late=decide(i,c,false);
  EXPECT_TRUE(late.acknowledged);EXPECT_FALSE(late.suppress);
  c.now+=.1;c.intent_wall_age=.1;
  const auto repeated=decide(i,c,false);
  EXPECT_TRUE(repeated.acknowledged);EXPECT_FALSE(repeated.suppress);
  c.now+=.5;c.intent_wall_age=.6;
  EXPECT_FALSE(decide(i,c,false).acknowledged);
}
TEST_F(PassingIntentTest, WaitingAckRefreshNeverOwnsActiveAbortOrNormalModules) {
  EXPECT_TRUE(refresh_waiting_candidate_ack(true,true,true,false,false));
  EXPECT_FALSE(refresh_waiting_candidate_ack(false,true,true,false,false));
  EXPECT_FALSE(refresh_waiting_candidate_ack(true,false,true,false,false));
  EXPECT_FALSE(refresh_waiting_candidate_ack(true,true,false,false,false));
  EXPECT_FALSE(refresh_waiting_candidate_ack(true,true,true,true,false));
  EXPECT_FALSE(refresh_waiting_candidate_ack(true,true,true,false,true));
}
TEST_F(PassingIntentTest, RequiresObservedOwner) {
  EXPECT_FALSE(decide(i,c,false).suppress);EXPECT_TRUE(decide(i,c,true).suppress);
}
TEST_F(PassingIntentTest, ArmAcknowledgesButNeverSuppresses) {
  i.phase="ARM";c.current_lane=1;
  EXPECT_TRUE(decide(i,c,false).acknowledged);EXPECT_FALSE(decide(i,c,true).suppress);
}
TEST_F(PassingIntentTest, RunningAndAbortAreUntouched) {
  c.running=true;EXPECT_FALSE(decide(i,c,true).suppress);
  c.running=false;c.aborting=true;EXPECT_FALSE(decide(i,c,true).suppress);
}
TEST_F(PassingIntentTest, MandatoryReserveOverridesHold) {
  c.remaining=149;EXPECT_FALSE(decide(i,c,true).acknowledged);
  c.remaining=260;c.return_buffer=260;EXPECT_FALSE(decide(i,c,true).suppress);
  c.return_buffer=60;c.near_terminal=true;EXPECT_FALSE(decide(i,c,true).suppress);
}
TEST_F(PassingIntentTest, RejectsStaleLifecycleAndInvalidGeometry) {
  c.now=11;EXPECT_FALSE(decide(i,c,true).acknowledged);c.now=10.1;
  i.route_stamp=2;EXPECT_FALSE(decide(i,c,true).acknowledged);i.route_stamp=1;
  i.target.insert(99);EXPECT_FALSE(decide(i,c,true).acknowledged);i.target.erase(99);
  i.target.insert(1);EXPECT_FALSE(decide(i,c,true).acknowledged);
}
TEST_F(PassingIntentTest, OnlyExactReturnDirectionIsDeferred) {
  c.direction="right";EXPECT_FALSE(decide(i,c,true).suppress);
  c.direction="left";c.target_lane=4;EXPECT_FALSE(decide(i,c,true).suppress);
}
TEST_F(PassingIntentTest, ActualHighSpeedBrakingAndLaneEndOverride) {
  c.speed=111./3.6;c.remaining=500;c.terminal_distance=114.8;
  EXPECT_GT(decide(i,c,true).reserve,c.speed*c.speed/(2*c.deceleration));
  EXPECT_FALSE(decide(i,c,true).suppress);
  c.terminal_distance=500;c.lateral_distance=114.8;
  EXPECT_FALSE(decide(i,c,true).suppress);
}
TEST_F(PassingIntentTest, InvalidBrakingFailsClosedAndPrepareNotDoubleCounted) {
  c.jerk=0;EXPECT_FALSE(decide(i,c,true).acknowledged);c.jerk=10;
  c.deceleration=0;EXPECT_FALSE(decide(i,c,true).acknowledged);c.deceleration=3;
  c.speed=10;c.return_buffer=200;
  EXPECT_DOUBLE_EQ(decide(i,c,true).reserve,209.0); // 200 + v*TTL + front, no extra prepare.
}
TEST_F(PassingIntentTest, JerkLimitedStopSupportsEarlyStopDuringRamp) {
  EXPECT_GT(stop_distance(30,2,3,10),30*30./6);
  EXPECT_GE(stop_distance(1,0,3,1),1./6);
  EXPECT_NEAR(stop_distance(30,-3,3,10),150,1e-9);
}
