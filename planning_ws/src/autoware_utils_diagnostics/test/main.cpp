#include <gtest/gtest.h>
#include <ros/ros.h>

class RosEnvironment : public testing::Environment
{
public:
  RosEnvironment(int argc, char ** argv) : argc_(argc), argv_(argv) {}
  void SetUp() override { ros::init(argc_, argv_, "autoware_utils_diagnostics_test"); }
  void TearDown() override { ros::shutdown(); }

private:
  int argc_;
  char ** argv_;
};

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  testing::AddGlobalTestEnvironment(new RosEnvironment(argc, argv));
  return RUN_ALL_TESTS();
}
