#include "autoware/behavior_path_lane_change_module/utils/generation_rejection.hpp"
#include "autoware/behavior_path_lane_change_module/utils/safety_snapshot.hpp"
#include "autoware/behavior_path_lane_change_module/utils/unapproved_candidate_diagnostic.hpp"
#include <gtest/gtest.h>
#include <array>
namespace bpp=autoware::behavior_path_planner;
namespace lc=bpp::utils::lane_change;

TEST(GenerationRejection, ExactFailedGeometryRoundTrip)
{
  bpp::PathWithLaneId prepare,reference;
  prepare.points.resize(2);reference.points.resize(3);
  prepare.points[0].point.pose.position.x=12.345;
  prepare.points[0].point.pose.orientation.w=1;
  prepare.points[0].lane_ids={42,43};
  reference.points[2].point.longitudinal_velocity_mps=0;
  bpp::ShiftedPath shifted;shifted.path=reference;shifted.shift_length={0,1.25,3.5};
  shifted.path.points[1].point.pose.position.y=-3.14159;
  bpp::LaneChangeInfo info{};info.duration.prepare=2.5;
  info.longitudinal_acceleration.lane_changing=-1;
  info.shift_line.end_idx=2;
  try {lc::reject_generation_yaw(true,"yaw rejected",prepare,shifted,reference,info,.12,.087);FAIL();}
  catch(const lc::GenerationRejection & error) {
    EXPECT_EQ(error.prepare,prepare);EXPECT_EQ(error.reference,reference);
    EXPECT_EQ(error.shifted.path,shifted.path);EXPECT_EQ(error.shifted.shift_length,shifted.shift_length);
    const auto restored=lc::decode_snapshot_ros_message<bpp::PathWithLaneId>(lc::snapshot_ros_message(error.shifted.path));
    EXPECT_EQ(restored,shifted.path);EXPECT_DOUBLE_EQ(error.info.duration.prepare,2.5);
    EXPECT_DOUBLE_EQ(error.info.longitudinal_acceleration.lane_changing,-1);
    EXPECT_EQ(error.info.shift_line.end_idx,2u);EXPECT_DOUBLE_EQ(error.yaw,.12);
  }
}
TEST(GenerationRejection, OffIsOriginalLogicErrorAndDiagnosticCannotPreventContinue)
{
  bpp::PathWithLaneId path;bpp::ShiftedPath shifted;bpp::LaneChangeInfo info{};
  bool continued=false;
  try {lc::reject_generation_yaw(false,"original rejection",path,shifted,path,info,.12,.087);}
  catch(const lc::GenerationRejection &) {FAIL()<<"OFF captured a payload";}
  catch(const std::logic_error & error) {EXPECT_STREQ(error.what(),"original rejection");continued=true;}
  EXPECT_TRUE(continued);
  int completed=0;
  for(int i=0;i<2;++i) {
    try {lc::reject_generation_yaw(true,"rejected",path,shifted,path,info,.12,.087);}
    catch(const lc::GenerationRejection &) {
      lc::generation_diagnostic_nonfatal([]{throw std::runtime_error("serialization failed");});
      ++completed;continue;
    }
    FAIL()<<"Rejected geometry escaped into candidate checks";
  }
  EXPECT_EQ(completed,2);
}
TEST(GenerationRejection, OneHertzAndOffDoesNotConsumeBudget)
{
  lc::GenerationDiagnosticRate rate;
  EXPECT_FALSE(rate.claim(false,10));EXPECT_TRUE(rate.claim(true,10));
  EXPECT_FALSE(rate.claim(true,10.999));EXPECT_TRUE(rate.claim(true,11));
}

TEST(GenerationRejection, ExactUpstreamValueCopyKeepsStopsHeaderAndLaneIds)
{
  bpp::PathWithLaneId path, upstream;
  bpp::ShiftedPath shifted; bpp::LaneChangeInfo info{};
  upstream.header.frame_id="map"; upstream.header.stamp=ros::Time(123,456);
  upstream.points.resize(4);
  upstream.points[1].point.longitudinal_velocity_mps=0;
  upstream.points[2].point.longitudinal_velocity_mps=12;
  upstream.points[1].lane_ids={201478,201479};
  upstream.points[3].point.pose.position.z=28.456;
  const auto original=upstream;
  try {
    lc::reject_generation_yaw(true,"rejected",path,shifted,path,info,.12,.087,&upstream);
    FAIL();
  } catch(const lc::GenerationRejection & error) {
    ASSERT_TRUE(error.upstream);
    upstream.points.clear(); // The failed-input packet owns its original bytes.
    EXPECT_EQ(*error.upstream,original);
    EXPECT_EQ(lc::decode_snapshot_ros_message<bpp::PathWithLaneId>(
      lc::snapshot_ros_message(*error.upstream)),original);
  }
  try {lc::reject_generation_yaw(true,"legacy",path,shifted,path,info,.12,.087);}
  catch(const lc::GenerationRejection & error) {EXPECT_FALSE(error.upstream);}
  try {lc::reject_generation_yaw(false,"off",path,shifted,path,info,.12,.087,&original);}
  catch(const lc::GenerationRejection &) {FAIL()<<"OFF captured upstream";}
  catch(const std::logic_error & error) {EXPECT_STREQ(error.what(),"off");}
}

TEST(UnapprovedCandidateDiagnostic, ExactCandidateInputsAndNativeOutcomeUnchanged)
{
  bpp::PathWithLaneId candidate;candidate.points.resize(2);
  candidate.points[0].point.pose.position.x=123.456;
  candidate.points[1].point.longitudinal_velocity_mps=0;
  candidate.points[1].lane_ids={200137,200138};
  const auto original=candidate;int calls=0;
  lc::audit_unapproved_candidate(true,true,false,true,[&]{
    ++calls;EXPECT_EQ(lc::decode_snapshot_ros_message<bpp::PathWithLaneId>(
      lc::snapshot_ros_message(candidate)),original);
  });
  EXPECT_EQ(calls,1);EXPECT_EQ(candidate,original);
  for(const auto flags:{std::array<bool,4>{false,true,false,true},
      {true,false,false,true},{true,true,true,true},{true,true,false,false}})
    lc::audit_unapproved_candidate(flags[0],flags[1],flags[2],flags[3],[&]{++calls;});
  EXPECT_EQ(calls,1);
  EXPECT_NO_THROW(lc::audit_unapproved_candidate(true,true,false,true,[]{throw 42;}));
  EXPECT_EQ(candidate,original);
}
