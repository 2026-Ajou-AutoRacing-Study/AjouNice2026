#pragma once
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <ros/ros.h>
#include <functional>
#include <vector>
namespace autoware::planning_test_manager {
class PlanningInterfaceTestManager {
 public:
  PlanningInterfaceTestManager();
  template<class T> void publishInput(ros::NodeHandle & target,const std::string & topic,const T & value,int repeats=3) const { ros::Publisher p; autoware::test_utils::createPublisherWithQoS<T>(target,topic,p); for(int i=0;i<30 && p.getNumSubscribers()==0;++i){ros::spinOnce();ros::WallDuration(0.01).sleep();} for(int i=0;i<repeats;++i){p.publish(value);ros::spinOnce();ros::WallDuration(0.01).sleep();} }
  template<class T,class C> void subscribeOutput(const std::string & topic,C callback) { subs_.push_back(node_.subscribe<T>(topic,10,callback)); }
  template<class T> void subscribeOutput(const std::string & topic) { subscribeOutput<T>(topic,[this](const typename T::ConstPtr &){++received_;}); }
  void normalTrajectory(ros::NodeHandle &,const std::string &); void abnormalTrajectory(ros::NodeHandle &,const std::string &); void normalRoute(ros::NodeHandle &,const std::string &); void abnormalRoute(ros::NodeHandle &,const std::string &); void behaviorNormalRoute(ros::NodeHandle &,const std::string &); void behaviorGoalOnLeftRoute(ros::NodeHandle &,const std::string &); void normalPathWithLaneId(ros::NodeHandle &,const std::string &); void abnormalPathWithLaneId(ros::NodeHandle &,const std::string &); void normalPath(ros::NodeHandle &,const std::string &); void abnormalPath(ros::NodeHandle &,const std::string &); void offTrackInitialPoses(ros::NodeHandle &,const std::string &); void offTrackOdometry(ros::NodeHandle &,const std::string &);
  void resetReceivedTopicNum(){received_=0;} size_t getReceivedTopicNum()const{return received_;}
 private: ros::NodeHandle node_; std::vector<ros::Subscriber> subs_; size_t received_{0};
}; }
