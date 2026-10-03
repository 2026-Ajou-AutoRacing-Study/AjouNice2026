#include "autoware/behavior_path_planner/final_ego_poll.hpp"
#include "../include/autoware/behavior_path_planner/coherent_ego_poll.hpp"
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <gtest/gtest.h>
#include <memory>

namespace {
template<class Ptr> struct ScriptedPoll {
  std::vector<Ptr> messages; size_t calls{0};
  Ptr take_data(){return messages.empty()?Ptr{}:messages[std::min(calls++,messages.size()-1)];}
};
template<class T> typename T::ConstPtr real_message(double stamp) {
  typename T::Ptr m(new T);m->header.stamp=ros::Time(stamp);m->header.frame_id="map";return m;
}
TEST(CoherentEgoPoll, ActualR48CachedThenRealNewerPreservesObjects)
{
  auto odom=real_message<nav_msgs::Odometry>(53.294953108);
  auto accel=real_message<geometry_msgs::AccelWithCovarianceStamped>(53.3149433);
  auto raw=real_message<nav_msgs::Odometry>(53.310879469);const auto identity=raw.get();
  ScriptedPoll<decltype(odom)> os{{odom,real_message<nav_msgs::Odometry>(53.3149433)}};
  ScriptedPoll<decltype(accel)> as{{accel}};
  EXPECT_TRUE(autoware::behavior_path_planner::coherent_ego_poll(os,as,odom,accel,raw,
    []{return ros::Time(53.3294594);},[]{return true;}));
  EXPECT_EQ(os.calls,2u);EXPECT_EQ(raw.get(),identity);
  EXPECT_EQ(odom->header.stamp,ros::Time(53.3149433));
}
TEST(CoherentEgoPoll, DiagnosticReportsActualRejectedPairWithoutChangingPolicy)
{
  auto odom=real_message<nav_msgs::Odometry>(10.);
  auto accel=real_message<geometry_msgs::AccelWithCovarianceStamped>(10.02);
  auto raw=real_message<nav_msgs::Odometry>(10.02);
  ScriptedPoll<decltype(odom)> os{{odom}};
  ScriptedPoll<decltype(accel)> as{{accel}};
  autoware::behavior_path_planner::CoherentEgoPollDiagnostic d;
  EXPECT_FALSE(autoware::behavior_path_planner::coherent_ego_poll(os,as,odom,accel,raw,
    []{return ros::Time(10.03);},[]{return true;},4,&d));
  EXPECT_STREQ(d.reason,"future_objects");EXPECT_EQ(d.attempts,4u);
  EXPECT_DOUBLE_EQ(d.acquired_at,10.03);
  EXPECT_EQ(odom->header.stamp,ros::Time(10.));EXPECT_EQ(raw->header.stamp,ros::Time(10.02));
  odom=real_message<nav_msgs::Odometry>(10.03);
  accel=real_message<geometry_msgs::AccelWithCovarianceStamped>(9.);
  EXPECT_FALSE(autoware::behavior_path_planner::coherent_ego_poll(os,as,odom,accel,raw,
    []{return ros::Time(10.04);},[]{return true;},4,&d));
  EXPECT_STREQ(d.reason,"accel_wall_age");EXPECT_EQ(d.attempts,0u);
}
TEST(CoherentEgoPoll, FiniteBudgetAndNoOldRawFallback)
{
  auto odom=real_message<nav_msgs::Odometry>(10.);auto raw=real_message<nav_msgs::Odometry>(10.02);
  auto accel=real_message<geometry_msgs::AccelWithCovarianceStamped>(10.);
  ScriptedPoll<decltype(odom)> os{{odom}};ScriptedPoll<decltype(accel)> as{{accel}};
  EXPECT_FALSE(autoware::behavior_path_planner::coherent_ego_poll(os,as,odom,accel,raw,
    []{return ros::Time(10.03);},[]{return true;}));EXPECT_EQ(os.calls,4u);
  EXPECT_EQ(raw->header.stamp,ros::Time(10.02));
  os.calls=0;
  EXPECT_FALSE(autoware::behavior_path_planner::coherent_ego_poll(os,as,odom,accel,raw,
    []{return ros::Time(10.03);},[]{return false;}));EXPECT_EQ(os.calls,0u);
}
TEST(CoherentEgoPoll, CoherentNoRepollAndStaleAccelRejected)
{
  auto odom=real_message<nav_msgs::Odometry>(10.03);auto raw=real_message<nav_msgs::Odometry>(10.02);
  auto accel=real_message<geometry_msgs::AccelWithCovarianceStamped>(9.);
  ScriptedPoll<decltype(odom)> os{{odom}};ScriptedPoll<decltype(accel)> as{{accel}};
  EXPECT_FALSE(autoware::behavior_path_planner::coherent_ego_poll(os,as,odom,accel,raw,
    []{return ros::Time(10.04);},[]{return true;}));EXPECT_EQ(os.calls,0u);
  accel=real_message<geometry_msgs::AccelWithCovarianceStamped>(10.03);
  EXPECT_TRUE(autoware::behavior_path_planner::coherent_ego_poll(os,as,odom,accel,raw,
    []{return ros::Time(10.04);},[]{return true;}));EXPECT_EQ(os.calls,0u);
}
TEST(CoherentEgoPoll, TimestampRegressionCannotReplaceActualNewest)
{
  auto odom=real_message<nav_msgs::Odometry>(10.);auto raw=real_message<nav_msgs::Odometry>(10.02);
  auto accel=real_message<geometry_msgs::AccelWithCovarianceStamped>(10.);
  ScriptedPoll<decltype(odom)> os{{real_message<nav_msgs::Odometry>(9.)}};
  ScriptedPoll<decltype(accel)> as{{real_message<geometry_msgs::AccelWithCovarianceStamped>(9.)}};
  EXPECT_FALSE(autoware::behavior_path_planner::coherent_ego_poll(os,as,odom,accel,raw,
    []{return ros::Time(10.03);},[]{return true;}));
  EXPECT_EQ(odom->header.stamp,ros::Time(10.));EXPECT_EQ(accel->header.stamp,ros::Time(10.));
}
using Odom = nav_msgs::Odometry;
using Accel = geometry_msgs::AccelWithCovarianceStamped;
using ObjectStamp = std_msgs::Header;
using autoware::behavior_path_planner::final_ego_poll;
template<class T> using Subscriber = autoware_utils_rclcpp::InterProcessPollingSubscriber<T>;
class FinalEgoPoll : public ::testing::Test {
protected:
  std::unique_ptr<ros::NodeHandle> node;
  std::unique_ptr<Subscriber<Odom>> odom;
  std::unique_ptr<Subscriber<Accel>> accel;
  std::unique_ptr<Subscriber<ObjectStamp>> objects;
  ros::Publisher po,pa,pp;
  void SetUp() override {
    if(!ros::isInitialized()) {
      int argc=0; char **argv=nullptr;
      ros::init(argc,argv,"test_bpp_final_ego_poll",ros::init_options::AnonymousName);
    }
    node=std::make_unique<ros::NodeHandle>("~");
    odom=std::make_unique<Subscriber<Odom>>(node.get(),"odom");
    accel=std::make_unique<Subscriber<Accel>>(node.get(),"accel");
    objects=std::make_unique<Subscriber<ObjectStamp>>(node.get(),"objects");
    po=node->advertise<Odom>("odom",1,true);pa=node->advertise<Accel>("accel",1,true);
    pp=node->advertise<ObjectStamp>("objects",1,true);
    const auto deadline=ros::WallTime::now()+ros::WallDuration(2);
    while((!po.getNumSubscribers()||!pa.getNumSubscribers()||!pp.getNumSubscribers())&&
      ros::WallTime::now()<deadline)ros::WallDuration(.005).sleep();
    ASSERT_GT(po.getNumSubscribers(),0u);ASSERT_GT(pa.getNumSubscribers(),0u);
  }
  void publish(double time) {
    Odom o;o.header.stamp=ros::Time(time);o.header.frame_id="map";o.pose.pose.position.x=time;
    Accel a;a.header=o.header;a.accel.accel.linear.x=time;
    po.publish(o);pa.publish(a);ros::WallDuration(.02).sleep();
  }
};

TEST_F(FinalEgoPoll, ArrivingWhileOtherInputServicedRefreshesBothWithoutChangingObjects) {
  publish(1.0);
  auto selected_odom=odom->take_data();auto selected_accel=accel->take_data();
  ASSERT_TRUE(selected_odom);ASSERT_TRUE(selected_accel);
  ObjectStamp object;object.stamp=ros::Time(1.015);object.frame_id="map";
  pp.publish(object);
  publish(1.02); // The initial ego poll already happened; other inputs take time.
  const auto selected_object=objects->take_data();ASSERT_TRUE(selected_object);
  EXPECT_TRUE(final_ego_poll(true,*odom,*accel,selected_odom,selected_accel));
  EXPECT_DOUBLE_EQ(selected_odom->header.stamp.toSec(),1.02);
  EXPECT_DOUBLE_EQ(selected_accel->header.stamp.toSec(),1.02);
  EXPECT_DOUBLE_EQ(selected_object->stamp.toSec(),1.015);
  EXPECT_EQ(objects->take_data(),selected_object);
  EXPECT_NEAR((selected_odom->header.stamp-selected_object->stamp).toSec(),.005,1e-9);
}
TEST_F(FinalEgoPoll, NoNewEgoRetainsCachedPointersAndSourceStamps) {
  publish(2.0);auto o=odom->take_data();auto a=accel->take_data();
  ASSERT_TRUE(o);ASSERT_TRUE(a);const auto old_o=o;const auto old_a=a;
  EXPECT_FALSE(final_ego_poll(true,*odom,*accel,o,a));
  EXPECT_EQ(o,old_o);EXPECT_EQ(a,old_a);EXPECT_DOUBLE_EQ(o->header.stamp.toSec(),2.0);
}
TEST_F(FinalEgoPoll, DisabledDoesNotConsumeNewEgoQueue) {
  publish(3.0);auto o=odom->take_data();auto a=accel->take_data();
  ASSERT_TRUE(o);ASSERT_TRUE(a);publish(3.02);
  EXPECT_FALSE(final_ego_poll(false,*odom,*accel,o,a));
  EXPECT_DOUBLE_EQ(o->header.stamp.toSec(),3.0);
  EXPECT_DOUBLE_EQ(a->header.stamp.toSec(),3.0);
  EXPECT_TRUE(final_ego_poll(true,*odom,*accel,o,a));
  EXPECT_DOUBLE_EQ(o->header.stamp.toSec(),3.02);
}
TEST_F(FinalEgoPoll, FutureObjectWithoutNewEgoIsNotRetimestampedOrDiscarded) {
  publish(4.0);auto o=odom->take_data();auto a=accel->take_data();
  ObjectStamp msg;msg.stamp=ros::Time(4.02);pp.publish(msg);ros::WallDuration(.02).sleep();
  auto object=objects->take_data();ASSERT_TRUE(object);
  EXPECT_FALSE(final_ego_poll(true,*odom,*accel,o,a));
  EXPECT_NEAR((o->header.stamp-object->stamp).toSec(),-.02,1e-9);
  EXPECT_EQ(objects->take_data(),object); // Existing strict skew rejection remains required.
}
} // namespace
