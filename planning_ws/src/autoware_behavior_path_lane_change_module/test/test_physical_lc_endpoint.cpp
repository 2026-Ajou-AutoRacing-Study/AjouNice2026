#include "autoware/behavior_path_lane_change_module/utils/physical_lc_endpoint.hpp"
#include <gtest/gtest.h>
#include <limits>
namespace e=autoware::behavior_path_planner::utils::lane_change::physical_endpoint;
namespace b=autoware::behavior_path_planner;
e::Path reference() {
  e::Path p;
  for(int i=0;i<12;++i) {
    decltype(p.points)::value_type q;q.point.pose.position.x=i;q.point.pose.orientation.w=1;
    q.point.longitudinal_velocity_mps=10;q.lane_ids={1};p.points.push_back(q);
  }
  return p;
}
b::ShiftedPath native(const e::Path&p) {
  b::PathShifter shifter;shifter.setPath(p);b::ShiftLine line;
  line.start=p.points[0].point.pose;line.end=p.points[6].point.pose;line.end_shift_length=1;
  shifter.addShiftLine(line);shifter.setVelocity(3);shifter.setLongitudinalAcceleration(0);
  shifter.setLateralAccelerationLimit(1);b::ShiftedPath out;
  if(!shifter.generate(&out,false))throw std::logic_error("fixture generation failed");
  return out;
}
b::LaneChangePath candidate(const e::Path&p) {
  b::LaneChangePath c;c.shifted_path=native(p);c.path=c.shifted_path.path;
  c.info.shift_line.start_idx=0;c.info.shift_line.end_idx=6;
  c.info.lane_changing_end=p.points[6].point.pose;c.info.lane_changing_end.position.y=.314;
  c.info.shift_line.end=c.info.lane_changing_end;
  c.info.duration.prepare=3;c.info.duration.lane_changing=2;
  c.info.length.prepare=10;c.info.length.lane_changing=6;
  return c;
}
TEST(PhysicalEndpoint, RealNativeShiftExactEndAndNoMutation) {
  auto p=reference();auto s=native(p);const auto before=s.path;
  const auto c=e::certify(p,s,s.path,0,6);
  EXPECT_EQ(c.reference_index,6);EXPECT_EQ(c.shifted_index,6);
  EXPECT_EQ(c.pose.position.x,6);EXPECT_EQ(c.pose.position.y,0);EXPECT_EQ(s.path,before);
}
TEST(PhysicalEndpoint, RemovalBeforeEndMustNotDriftIndex) {
  auto p=reference();auto s=native(p);s.path.points.erase(s.path.points.begin()+2);
  auto c=e::certify(p,s,s.path,0,6);EXPECT_EQ(c.shifted_index,5);EXPECT_EQ(c.pose.position.x,6);
}
TEST(PhysicalEndpoint, RealNativeOverlapRemovalBeforeEnd) {
  auto p=reference();p.points.insert(p.points.begin()+2,p.points[2]);auto s=native(p);
  ASSERT_LT(s.path.points.size(),p.points.size());auto c=e::certify(p,s,s.path,0,6);
  EXPECT_EQ(c.reference_index,6);EXPECT_EQ(c.shifted_index,5);EXPECT_EQ(c.pose.position.x,5);
}
TEST(PhysicalEndpoint, MissingEndpointRejectedWithoutProjection) {
  auto p=reference();auto s=native(p);s.path.points[6].point.pose.position.y=.00000001;
  EXPECT_THROW(e::certify(p,s,s.path,0,6),std::logic_error);
}
TEST(PhysicalEndpoint, NonzeroShiftRejected) {
  auto p=reference();auto s=native(p);s.shift_length[6]=1e-12;
  EXPECT_THROW(e::certify(p,s,s.path,0,6),std::logic_error);
}
TEST(PhysicalEndpoint, WrongFollowingGeometryRejected) {
  auto p=reference();auto s=native(p);s.path.points[7].point.pose.position.y=.1;
  EXPECT_THROW(e::certify(p,s,s.path,0,6),std::logic_error);
}
TEST(PhysicalEndpoint, DuplicateIdentityRejected) {
  auto p=reference();auto s=native(p);s.path.points.push_back(s.path.points[6]);
  EXPECT_THROW(e::certify(p,s,s.path,0,6),std::logic_error);
}
TEST(PhysicalEndpoint, CombinedPathMustPreserveSameEndpoint) {
  auto p=reference();auto s=native(p);auto combined=s.path;
  combined.points.insert(combined.points.begin(),s.path.points[0]);
  auto c=e::certify(p,s,combined,0,6);EXPECT_EQ(c.combined_index,7);
  combined.points[7].point.pose.position.z+=.001;
  EXPECT_THROW(e::certify(p,s,combined,0,6),std::logic_error);
}
TEST(PhysicalEndpoint, NonfiniteOrInvalidIndicesRejected) {
  auto p=reference();auto s=native(p);
  EXPECT_THROW(e::certify(p,s,s.path,6,6),std::logic_error);
  s.shift_length[7]=NAN;
  EXPECT_THROW(e::certify(p,s,s.path,0,6),std::logic_error);
}
TEST(PhysicalEndpoint, OnlyEndMetadataChangesForBothShifterSpeedModels) {
  for(const auto type:{b::lane_change::PathType::ConstantJerk,b::lane_change::PathType::SpatialSpeedProfile}) {
    const auto p=reference();auto c=candidate(p);c.type=type;const auto before=c;
    e::apply(p,c);
    EXPECT_EQ(c.path,before.path);EXPECT_EQ(c.shifted_path.path,before.shifted_path.path);
    EXPECT_EQ(c.shifted_path.shift_length,before.shifted_path.shift_length);
    EXPECT_EQ(c.type,before.type);EXPECT_EQ(c.info.shift_line.start_idx,0);EXPECT_EQ(c.info.shift_line.end_idx,6);
    EXPECT_EQ(c.info.duration.prepare,3);EXPECT_EQ(c.info.duration.lane_changing,2);
    EXPECT_EQ(c.info.length.prepare,10);EXPECT_EQ(c.info.length.lane_changing,6);
    EXPECT_EQ(c.info.lane_changing_end,c.path.points[6].point.pose);
    EXPECT_EQ(c.info.shift_line.end,c.info.lane_changing_end);
  }
}
TEST(PhysicalEndpoint, FailureIsAtomicAndFrenetIsUnchanged) {
  auto p=reference();auto c=candidate(p);c.shifted_path.shift_length[6]=.1;
  const auto before=c.info.lane_changing_end;
  EXPECT_THROW(e::apply(p,c),std::logic_error);
  EXPECT_EQ(c.info.lane_changing_end,before);EXPECT_EQ(c.info.shift_line.end,before);
  c=candidate(p);c.type=b::lane_change::PathType::FrenetPlanner;
  EXPECT_THROW(e::apply(p,c),std::logic_error);
  EXPECT_EQ(c.info.lane_changing_end,before);
}
TEST(PhysicalEndpoint, SpeedPhaseEndpointMismatchDoesNotRetagMetadata) {
  const auto p=reference();auto c=candidate(p);const auto old_end=c.info.lane_changing_end;
  EXPECT_THROW(e::apply(p,c,5),std::logic_error);
  EXPECT_EQ(c.info.lane_changing_end,old_end);
  EXPECT_NO_THROW(e::apply(p,c,6));
}
TEST(PhysicalEndpoint, SizeMaxIndicesRejectedBeforeArithmetic) {
  auto p=reference();auto s=native(p);const auto huge=std::numeric_limits<size_t>::max();
  EXPECT_THROW(e::certify(p,s,s.path,huge,6),std::logic_error);
  EXPECT_THROW(e::certify(p,s,s.path,0,huge),std::logic_error);
}
