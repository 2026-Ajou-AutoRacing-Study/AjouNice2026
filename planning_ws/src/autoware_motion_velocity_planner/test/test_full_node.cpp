#include "node.hpp"
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>
#include <ros/package.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <gtest/gtest.h>
#include <cstdlib>
#include <future>

namespace {
using namespace autoware::motion_velocity_planner;
class FullMvpNode :public testing::Test {
protected:
  ros::NodeHandle handle{"/full_mvp_node_test"};
  std::unique_ptr<MotionVelocityPlannerNode> node;
  ros::Publisher trajectory_pub,objects_pub,cloud_pub,odom_pub,accel_pub,map_pub,signals_pub;
  ros::Subscriber output_sub;
  autoware_planning_msgs::Trajectory received,input;
  size_t count=0;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf;
  void SetUp() override {
    ASSERT_EQ(
      std::system(
        "rosparam load \"$(rospack find autoware_motion_velocity_planner)/config/ros1/"
        "motion_velocity_planner.yaml\" /full_mvp_node_test"),
      0);
    ASSERT_EQ(
      std::system(
        "rosparam load \"$(rospack find autoware_motion_velocity_planner)/config/ros1/"
        "obstacle_stop_default.yaml\" /full_mvp_node_test"),
      0);
    ASSERT_EQ(
      std::system(
        "rosparam load \"$(rospack find autoware_velocity_smoother)/config/ros1/"
        "default_common.yaml\" /full_mvp_node_test"),
      0);
    ASSERT_EQ(
      std::system(
        "rosparam load \"$(rospack find autoware_velocity_smoother)/config/ros1/"
        "default_velocity_smoother.yaml\" /full_mvp_node_test"),
      0);
    ASSERT_EQ(
      std::system(
        "rosparam load \"$(rospack find autoware_velocity_smoother)/config/ros1/"
        "Analytical.yaml\" /full_mvp_node_test"),
      0);
    ASSERT_EQ(
      std::system(
        "rosparam load \"$(rospack find autoware_launch)/config/ros1/planning/"
        "scenario_planning/common/nearest_search.param.yaml\" /full_mvp_node_test"),
      0);
    for (const auto & entry : {
           std::pair{"wheel_radius", 0.39}, {"wheel_width", 0.42}, {"wheel_base", 2.74},
           {"wheel_tread", 1.63}, {"front_overhang", 1.0}, {"rear_overhang", 1.03},
           {"left_overhang", 0.1}, {"right_overhang", 0.1}, {"vehicle_height", 2.5},
           {"max_steer_angle", 0.70}}) {
      handle.setParam(entry.first, entry.second);
    }
    handle.setParam("smooth_velocity_before_planning",true);
    handle.setParam("launch_modules",std::vector<std::string>{"autoware::motion_velocity_planner::ObstacleStopModule"});
    node=std::make_unique<MotionVelocityPlannerNode>(handle);
    trajectory_pub=handle.advertise<Trajectory>("input/trajectory",1,true);
    objects_pub=handle.advertise<autoware_perception_msgs::PredictedObjects>("input/dynamic_objects",1,true);
    cloud_pub=handle.advertise<sensor_msgs::PointCloud2>("input/no_ground_pointcloud",1,true);
    odom_pub=handle.advertise<nav_msgs::Odometry>("input/vehicle_odometry",1,true);
    accel_pub=handle.advertise<geometry_msgs::AccelWithCovarianceStamped>("input/accel",1,true);
    map_pub=handle.advertise<autoware_map_msgs::LaneletMapBin>("input/vector_map",1,true);
    signals_pub=handle.advertise<autoware_perception_msgs::TrafficLightGroupArray>("input/traffic_signals",1,true);
    output_sub=handle.subscribe<Trajectory>("output/trajectory",1,[this](const Trajectory::ConstPtr & msg){received=*msg;++count;});
    pump(0.1);
  }
  void TearDown() override {node.reset();}
  void pump(double seconds) {
    const auto end=ros::WallTime::now()+ros::WallDuration(seconds);
    while(ros::WallTime::now()<end) {ros::spinOnce();ros::WallDuration(0.005).sleep();}
  }
  void loadFixture(bool traffic=false,bool publish_map=true,bool publish_tf=true) {
    const auto share=ros::package::getPath("autoware_motion_velocity_planner");
    const auto config=YAML::LoadFile(share+"/test_data/"+(traffic?"test_traffic_light.yaml":"subscribed_topics.yaml"));
    const auto path=autoware::test_utils::parse<autoware_internal_planning_msgs::PathWithLaneId>(config["path_with_lane_id"]);
    input.header=path.header;
    for(const auto & p:path.points) {
      autoware_planning_msgs::TrajectoryPoint point;
      point.pose=p.point.pose; point.longitudinal_velocity_mps=p.point.longitudinal_velocity_mps;
      point.lateral_velocity_mps=p.point.lateral_velocity_mps;point.heading_rate_rps=p.point.heading_rate_rps;
      input.points.push_back(point);
    }
    objects_pub.publish(autoware::test_utils::parse<autoware_perception_msgs::PredictedObjects>(config["dynamic_object"]));
    odom_pub.publish(autoware::test_utils::parse<nav_msgs::Odometry>(config["self_odometry"]));
    accel_pub.publish(autoware::test_utils::parse<geometry_msgs::AccelWithCovarianceStamped>(config["self_acceleration"]));
    signals_pub.publish(autoware::test_utils::parse<autoware_perception_msgs::TrafficLightGroupArray>(config["traffic_signal"]));
    pcl::PointCloud<pcl::PointXYZ> empty;
    sensor_msgs::PointCloud2 cloud; pcl::toROSMsg(empty,cloud);
    cloud.header.frame_id="full_mvp_cloud_frame";cloud.header.stamp=ros::Time::now();cloud_pub.publish(cloud);
    if(publish_map) {
      const auto map_path=traffic?ros::package::getPath("autoware_test_utils")+"/test_map/intersection/lanelet2_map.osm":share+"/test_map/lanelet2_map.osm";
      map_pub.publish(autoware::test_utils::make_map_bin_msg(map_path));
    }
    if(publish_tf) {
      tf=std::make_unique<tf2_ros::StaticTransformBroadcaster>();
      geometry_msgs::TransformStamped transform;transform.header.frame_id="map";
      transform.child_frame_id="full_mvp_cloud_frame";transform.transform.rotation.w=1.0;
      transform.header.stamp=ros::Time::now();tf->sendTransform(transform);
    }
    pump(0.2);
  }
  bool waitOutput(double seconds=5.0) {
    const auto end=ros::WallTime::now()+ros::WallDuration(seconds);
    while(count==0 && ros::WallTime::now()<end) {trajectory_pub.publish(input);pump(0.05);}
    return count>0;
  }
  template<class Service> bool call(const std::string & name,Service & service) {
    auto client=handle.serviceClient<Service>(name);
    if(!client.waitForExistence(ros::Duration(1.0))) return false;
    auto result=std::async(std::launch::async,[&]{return client.call(service);});
    while(result.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready) pump(0.01);
    return result.get();
  }
};

TEST_F(FullMvpNode, OriginalRecordedMainScenarioProducesFiniteTrajectory) {
  EXPECT_EQ(count,0U);loadFixture();ASSERT_TRUE(waitOutput());
  ASSERT_FALSE(received.points.empty()); EXPECT_EQ(received.header.frame_id,input.header.frame_id);
  for(const auto & point:received.points) {
    EXPECT_TRUE(std::isfinite(point.pose.position.x));EXPECT_TRUE(std::isfinite(point.longitudinal_velocity_mps));
  }
}
TEST_F(FullMvpNode, OriginalRecordedTrafficLightScenarioProducesTrajectory) {
  EXPECT_EQ(count,0U);loadFixture(true);EXPECT_TRUE(waitOutput());EXPECT_TRUE(ros::ok());
}
TEST_F(FullMvpNode, OriginalParameterUpdateAndInvalidTypeDoNotCrashNode) {
  bool value=false;ASSERT_TRUE(handle.getParam("smooth_velocity_before_planning",value));EXPECT_TRUE(value);
  handle.setParam("smooth_velocity_before_planning",false);pump(0.3);
  ASSERT_TRUE(handle.getParam("smooth_velocity_before_planning",value));EXPECT_FALSE(value);
  EXPECT_FALSE(node->smoothVelocityBeforePlanning());
  handle.setParam("smooth_velocity_before_planning",std::string("invalid"));pump(0.3);
  EXPECT_TRUE(ros::ok());EXPECT_FALSE(node->smoothVelocityBeforePlanning());
  handle.setParam("smooth_velocity_before_planning",true);pump(0.3);EXPECT_TRUE(node->smoothVelocityBeforePlanning());
}
TEST_F(FullMvpNode, OriginalLoadServiceRespondsAndReportsMissingPlugin) {
  autoware_internal_planning_msgs::LoadPlugin request;request.request.plugin_name="test_plugin";
  ASSERT_TRUE(call("service/load_plugin",request));EXPECT_FALSE(request.response.success);EXPECT_EQ(node->loadedModuleCount(),1U);
}
TEST_F(FullMvpNode, OriginalUnloadServiceRespondsAndRealPluginCanReload) {
  autoware_internal_planning_msgs::UnloadPlugin unload;unload.request.plugin_name="test_plugin";
  ASSERT_TRUE(call("service/unload_plugin",unload));EXPECT_FALSE(unload.response.success);
  unload.request.plugin_name="autoware::motion_velocity_planner::ObstacleStopModule";
  ASSERT_TRUE(call("service/unload_plugin",unload));EXPECT_TRUE(unload.response.success);EXPECT_EQ(node->loadedModuleCount(),0U);
  autoware_internal_planning_msgs::LoadPlugin load;load.request.plugin_name=unload.request.plugin_name;
  ASSERT_TRUE(call("service/load_plugin",load));EXPECT_TRUE(load.response.success);EXPECT_EQ(node->loadedModuleCount(),1U);
}
TEST_F(FullMvpNode, MissingMapSuppressesOutput) {
  loadFixture(false,false);EXPECT_FALSE(waitOutput(0.3));
}
TEST_F(FullMvpNode, MissingInputsSuppressOutput) {
  input.points.resize(3);EXPECT_FALSE(waitOutput(0.3));
}
TEST_F(FullMvpNode, MissingCloudTransformSuppressesOutput) {
  loadFixture(false,true,false);
  sensor_msgs::PointCloud2 cloud;pcl::PointCloud<pcl::PointXYZ> empty;pcl::toROSMsg(empty,cloud);
  cloud.header.frame_id="deliberately_unconnected_cloud_frame";cloud.header.stamp=ros::Time::now();cloud_pub.publish(cloud);pump(0.1);
  EXPECT_FALSE(waitOutput(0.3));
}
TEST_F(FullMvpNode, ActualHostStopsForPedestrianAndPreservesClearTrajectory) {
  loadFixture();
  handle.setParam("smooth_velocity_before_planning",false);pump(0.3);
  input.points.clear();input.header.frame_id="map";
  for(size_t i=0;i<=60;++i) {
    autoware_planning_msgs::TrajectoryPoint point;point.pose.position.x=i;
    point.pose.orientation.w=1.0;point.longitudinal_velocity_mps=5.0;
    input.points.push_back(point);
  }
  nav_msgs::Odometry odom;odom.header.frame_id="map";odom.pose.pose.orientation.w=1.0;
  odom.twist.twist.linear.x=5.0;odom_pub.publish(odom);
  geometry_msgs::AccelWithCovarianceStamped accel;accel_pub.publish(accel);
  autoware_perception_msgs::PredictedObjects objects;objects.header.frame_id="map";
  objects.header.stamp=ros::Time::now();objects_pub.publish(objects);pump(0.1);
  ASSERT_TRUE(waitOutput());
  for(const auto & point:received.points) EXPECT_FLOAT_EQ(point.longitudinal_velocity_mps,5.0F);
  autoware_perception_msgs::PredictedObject person;
  person.existence_probability=1.0;person.classification.resize(1);
  person.classification[0].label=autoware_perception_msgs::ObjectClassification::PEDESTRIAN;
  person.classification[0].probability=1.0;
  person.kinematics.initial_pose_with_covariance.pose.position.x=20.0;
  person.kinematics.initial_pose_with_covariance.pose.orientation.w=1.0;
  person.shape.type=autoware_perception_msgs::Shape::CYLINDER;
  person.shape.dimensions.x=0.5;person.shape.dimensions.y=0.5;person.shape.dimensions.z=1.8;
  person.kinematics.predicted_paths.resize(1);
  auto & predicted=person.kinematics.predicted_paths.front();predicted.confidence=1.0;
  predicted.time_step.nanosec=500000000;
  predicted.path.assign(10,person.kinematics.initial_pose_with_covariance.pose);
  objects.objects.push_back(person);objects_pub.publish(objects);pump(0.1);
  const auto deadline=ros::WallTime::now()+ros::WallDuration(3.0);
  while(ros::WallTime::now()<deadline && received.points.back().longitudinal_velocity_mps!=0.0F) {
    trajectory_pub.publish(input);pump(0.05);
  }
  EXPECT_FLOAT_EQ(received.points.back().longitudinal_velocity_mps,0.0F);
  const auto stop=std::find_if(received.points.begin(),received.points.end(),[](const auto & point){return point.longitudinal_velocity_mps==0.0F;});
  ASSERT_NE(stop,received.points.end());EXPECT_LT(stop->pose.position.x,20.0);
}
}
int main(int argc,char ** argv) {
  ros::init(argc,argv,"full_mvp_node_tests");ros::NodeHandle owner;
  testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
