#include "lane_change_ownership.hpp"
#include <gtest/gtest.h>
using namespace autoware::motion_velocity_planner;
namespace {
struct Fixture {
  autoware_internal_planning_msgs::PathWithLaneId commitment;
  std::vector<autoware_planning_msgs::TrajectoryPoint> trajectory;
  geometry_msgs::Pose ego;
  Fixture() {
    commitment.header.frame_id="map"; commitment.header.stamp=ros::Time(100);
    for(int i=0;i<=100;++i) {
      autoware_internal_planning_msgs::PathPointWithLaneId p;
      p.point.pose.position.x=i; p.point.longitudinal_velocity_mps=15;
      commitment.points.push_back(p);
      autoware_planning_msgs::TrajectoryPoint t;
      t.pose=p.point.pose; t.longitudinal_velocity_mps=15;
      trajectory.push_back(t);
    }
  }
  std::string check(ros::Time now=ros::Time(100)) {
    return lane_change_ownership_rejection(commitment,ros::Time(100),now,trajectory,ego);
  }
};
}
TEST(LaneChangeOwnership, OnlyMatchingApprovedOutputOwnsSpeed) { Fixture f; EXPECT_TRUE(f.check().empty()); }
TEST(LaneChangeOwnership, WaitingOrCompletedHasNoLease) { Fixture f; f.commitment.points.clear(); EXPECT_EQ(f.check(),"no_approved_lc_output"); }
TEST(LaneChangeOwnership, ExpiryAndClockResetRevokeLease) { Fixture f; EXPECT_EQ(f.check(ros::Time(101)),"commitment_expired"); EXPECT_EQ(f.check(ros::Time(99)),"commitment_expired"); }
TEST(LaneChangeOwnership, DifferentManeuverGeometryIsRejected) { Fixture f; f.trajectory[20].pose.position.y=1; EXPECT_EQ(f.check(),"lc_geometry_changed"); }
TEST(LaneChangeOwnership, DownstreamSpeedChangeIsDetected) { Fixture f; f.trajectory[20].longitudinal_velocity_mps=10; EXPECT_EQ(f.check(),"lc_speed_changed"); }
TEST(LaneChangeOwnership, NonFiniteSpeedCannotOwn) { Fixture f; f.trajectory[20].longitudinal_velocity_mps=std::numeric_limits<double>::quiet_NaN(); EXPECT_EQ(f.check(),"lc_speed_changed"); }
TEST(LaneChangeOwnership, WrongFrameIsRejected) { Fixture f; f.commitment.header.frame_id="odom"; EXPECT_EQ(f.check(),"commitment_frame_mismatch"); }
int main(int argc, char ** argv) { testing::InitGoogleTest(&argc,argv); return RUN_ALL_TESTS(); }
