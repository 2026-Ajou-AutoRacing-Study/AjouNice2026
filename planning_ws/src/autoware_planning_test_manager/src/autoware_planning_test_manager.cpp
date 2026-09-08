#include <autoware/planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <nav_msgs/Odometry.h>
namespace a=autoware::planning_test_manager;
a::PlanningInterfaceTestManager::PlanningInterfaceTestManager():node_(){}
void a::PlanningInterfaceTestManager::normalTrajectory(ros::NodeHandle &n,const std::string&t){publishInput(n,t,autoware::test_utils::generateTrajectory<autoware_planning_msgs::Trajectory>(10,1.0),5);}
void a::PlanningInterfaceTestManager::abnormalTrajectory(ros::NodeHandle &n,const std::string&t){publishInput(n,t,autoware_planning_msgs::Trajectory{},5);}
void a::PlanningInterfaceTestManager::normalRoute(ros::NodeHandle &n,const std::string&t){publishInput(n,t,autoware::test_utils::makeNormalRoute(),5);}
void a::PlanningInterfaceTestManager::abnormalRoute(ros::NodeHandle &n,const std::string&t){publishInput(n,t,autoware_planning_msgs::LaneletRoute{},5);}
void a::PlanningInterfaceTestManager::behaviorNormalRoute(ros::NodeHandle &n,const std::string&t){publishInput(n,t,autoware::test_utils::makeBehaviorNormalRoute(),5);}
void a::PlanningInterfaceTestManager::behaviorGoalOnLeftRoute(ros::NodeHandle &n,const std::string&t){publishInput(n,t,autoware::test_utils::makeBehaviorGoalOnLeftSideRoute(),5);}
void a::PlanningInterfaceTestManager::normalPathWithLaneId(ros::NodeHandle &n,const std::string&t){publishInput(n,t,autoware::test_utils::loadPathWithLaneIdInYaml(),5);}
void a::PlanningInterfaceTestManager::abnormalPathWithLaneId(ros::NodeHandle &n,const std::string&t){publishInput(n,t,autoware_internal_planning_msgs::PathWithLaneId{},5);}
void a::PlanningInterfaceTestManager::normalPath(ros::NodeHandle &n,const std::string&t){publishInput(n,t,autoware::motion_utils::convertToPath(autoware::test_utils::loadPathWithLaneIdInYaml()),5);}
void a::PlanningInterfaceTestManager::abnormalPath(ros::NodeHandle &n,const std::string&t){publishInput(n,t,autoware_planning_msgs::Path{},5);}
void a::PlanningInterfaceTestManager::offTrackInitialPoses(ros::NodeHandle &n,const std::string&t){for(double d:{0.,1.,10.,100.}) publishInput(n,t,autoware::test_utils::makeInitialPose(d),5);}
void a::PlanningInterfaceTestManager::offTrackOdometry(ros::NodeHandle &n,const std::string&t){for(double d:{0.,1.,10.,100.}) publishInput(n,t,autoware::test_utils::makeOdometry(d),5);}
