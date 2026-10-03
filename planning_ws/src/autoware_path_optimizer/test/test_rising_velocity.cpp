#include "autoware/motion_utils/trajectory/rising_velocity.hpp"
#include <gtest/gtest.h>
#include <limits>
using autoware::motion_utils::rising_velocity_at_projection;
namespace {
geometry_msgs::Point point(double x,double y=0,double z=0)
{geometry_msgs::Point p;p.x=x;p.y=y;p.z=z;return p;}
}
TEST(RisingVelocity, ConstantAccelerationHasNoHeldLowSpeedPlateau)
{
  const auto a=point(0),b=point(2);
  for(double x:{0.,.1,.5,1.,1.5,2.}) {
    const double v=rising_velocity_at_projection(a,b,point(x),2.,4.);
    EXPECT_NEAR(v*v,4.+6.*x,1e-12);
    EXPECT_GE(v,2.);EXPECT_LE(v,4.);
  }
}
TEST(RisingVelocity, StopsAndDescendingCapsRemainLegacy)
{
  for(auto speeds:{std::pair<double,double>{0.,10.},{10.,0.},{4.,2.},{3.,3.},{-2.,4.}})
    EXPECT_DOUBLE_EQ(rising_velocity_at_projection(point(0),point(2),point(1),
      speeds.first,speeds.second),speeds.first);
}
TEST(RisingVelocity, PlanarProjectionClampsAndPreservesHeightSemantics)
{
  const auto a=point(0,0,28),b=point(2,0,30);
  EXPECT_DOUBLE_EQ(rising_velocity_at_projection(a,b,point(-1,4),2,4),2);
  EXPECT_DOUBLE_EQ(rising_velocity_at_projection(a,b,point(3,4),2,4),4);
  EXPECT_NEAR(rising_velocity_at_projection(a,b,point(1,4,0),2,4),std::sqrt(10.),1e-12);
}
TEST(RisingVelocity, DegenerateAndInvalidGeometryUsesOriginalValue)
{
  EXPECT_DOUBLE_EQ(rising_velocity_at_projection(point(0),point(0),point(1),2,4),2);
  EXPECT_DOUBLE_EQ(rising_velocity_at_projection(point(0),point(2),point(NAN),2,4),2);
  EXPECT_DOUBLE_EQ(rising_velocity_at_projection(point(0),point(2),point(1),2,
    std::numeric_limits<double>::infinity()),2);
}
int main(int argc,char **argv)
{
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
