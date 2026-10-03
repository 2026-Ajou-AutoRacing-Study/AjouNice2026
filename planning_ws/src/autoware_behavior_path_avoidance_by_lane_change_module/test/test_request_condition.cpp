#include "request_condition.hpp"
#include <gtest/gtest.h>
namespace request=autoware::behavior_path_planner::avoidance_request;
TEST(AvoidanceRequest, NoErrorRequiresBothSpecialTargetAndValidPath)
{
  EXPECT_TRUE(request::allowed(std::nullopt,true,true));
  EXPECT_FALSE(request::allowed(std::nullopt,false,true));
  EXPECT_FALSE(request::allowed(std::nullopt,true,false));
  EXPECT_FALSE(request::allowed(std::nullopt,false,false));
}
TEST(AvoidanceRequest, ErrorAlwaysRejectsRegardlessOfCandidate)
{
  for(bool special:{false,true})for(bool valid:{false,true})
    EXPECT_FALSE(request::allowed(std::string("route unavailable"),special,valid));
}
TEST(AvoidanceRequest, EmptyButEngagedErrorStillRejects)
{
  EXPECT_FALSE(request::allowed(std::string(),true,true));
}
