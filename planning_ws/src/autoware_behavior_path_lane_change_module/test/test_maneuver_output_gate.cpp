#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_lane_change_module/utils/current_lane_cancel.hpp"
#include "autoware/behavior_path_lane_change_module/utils/safety_snapshot.hpp"
#include <gtest/gtest.h>
#include <std_msgs/String.h>
#include <cstdlib>
#include "autoware/behavior_path_planner_common/interface/fresh_input_provider.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"
#include "autoware/behavior_path_lane_change_module/utils/bounded_speed_grid.hpp"
#include "autoware/behavior_path_lane_change_module/utils/hard_speed_envelope.hpp"
#include "autoware/behavior_path_lane_change_module/utils/prepared_safety_world.hpp"
#include "autoware/behavior_path_lane_change_module/utils/rss_positive_grid.hpp"
#include <autoware_map_loader/lanelet2_map_loader.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

namespace bpp = autoware::behavior_path_planner;
namespace {
class OutputProbe : public bpp::NormalLaneChange {
public:
  OutputProbe() : NormalLaneChange(std::make_shared<bpp::LaneChangeParameters>(),
      bpp::LaneChangeModuleType::NORMAL, bpp::Direction::LEFT) {}
  using NormalLaneChange::enforce_strict_maneuver_safety;
  using NormalLaneChange::publishStrictSafetySnapshot;
  using NormalLaneChange::check_strict_maneuver_safety;
  using NormalLaneChange::prepare_admission_world;
  std::shared_ptr<const bpp::PlannerData> privateData()const{return planner_data_;}
  auto privateCommon()const{return common_data_ptr_;}
  auto utilityStatusPath()const{return status_.lane_change_path.path;}
  void seedUtilityNativeGuards() {
    transient_is_approved_=false;
    auto & t=common_data_ptr_->transient_data;
    t.has_valid_lc_distance=true;t.current_dist_buffer={1,100};t.dist_to_terminal_end=1000;
    lane_change_parameters_->trajectory.min_lane_changing_velocity=5;
    lane_change_parameters_->backward_length_buffer_for_end_of_lane=5;
  }
  double limit(double v,double maximum)const{return common_data_ptr_->acceleration_limits.limit(v,maximum);}
  void configureFreshFixture(const boost::property_tree::ptree & packet) {
    namespace lc=bpp::utils::lane_change;
    auto rss=[&](const char * key){const auto&r=packet.get_child(key);bpp::utils::path_safety_checker::RSSparams p;
      p.front_vehicle_deceleration=r.get<double>("front_vehicle_deceleration");
      p.rear_vehicle_deceleration=r.get<double>("rear_vehicle_deceleration");
      p.rear_vehicle_reaction_time=r.get<double>("rear_vehicle_reaction_time");
      p.rear_vehicle_safety_time_margin=r.get<double>("rear_vehicle_safety_time_margin");
      p.lateral_distance_max_threshold=r.get<double>("lateral_distance_max_threshold");
      p.longitudinal_distance_min_threshold=r.get<double>("longitudinal_distance_min_threshold");
      p.longitudinal_velocity_delta_time=r.get<double>("longitudinal_velocity_delta_time");
      p.extended_polygon_policy=r.get<std::string>("extended_polygon_policy");return p;};
    lane_change_parameters_->safety.rss_params=rss("rss_execution");
    lane_change_parameters_->safety.rss_params_for_abort=rss("rss_cancel");
    lane_change_parameters_->safety.rss_params_for_prepare=rss("rss_prepare");
    lane_change_parameters_->safety.collision_check.prediction_time_resolution=packet.get<double>("limits.prediction_resolution");
    lane_change_parameters_->safety.collision_check.th_yaw_diff=packet.get<double>("limits.th_yaw_diff");
    lane_change_parameters_->trajectory.max_longitudinal_acc=packet.get<double>("limits.lc_max_acc");
    lane_change_parameters_->trajectory.min_longitudinal_acc=packet.get<double>("limits.lc_min_acc");
    lane_change_parameters_->trajectory.min_prepare_duration=packet.get<double>("limits.min_prepare_duration");
    lane_change_parameters_->trajectory.lat_acc_map.add(0,1,4.5);
    lane_change_parameters_->trajectory.lat_acc_map.add(100,1,4.5);
    for(const auto& row:packet.get_child("current_lanes"))common_data_ptr_->lanes_ptr->current.push_back(
      planner_data_->route_handler->getLaneletMapPtr()->laneletLayer.get(row.second.get<lanelet::Id>("id")));
    for(const auto& row:packet.get_child("target_lanes"))common_data_ptr_->lanes_ptr->target.push_back(
      planner_data_->route_handler->getLaneletMapPtr()->laneletLayer.get(row.second.get<lanelet::Id>("id")));
    common_data_ptr_->lanes_ptr->target_neighbor=common_data_ptr_->lanes_ptr->target;
    *common_data_ptr_->lanes_polygon_ptr=lc::create_lanes_polygon(common_data_ptr_);
    common_data_ptr_->current_lanes_path=lc::decode_snapshot_ros_message<bpp::PathWithLaneId>(packet.get_child("current_speed_reference"));
    common_data_ptr_->target_lanes_path=lc::decode_snapshot_ros_message<bpp::PathWithLaneId>(packet.get_child("target_speed_reference"));
    autoware::motion_utils::AccelerationLimits::Map values;
    for(const auto&r:packet.get_child("acceleration_map"))values.emplace_back(r.second.get<double>("speed"),r.second.get<double>("acceleration"));
    common_data_ptr_->acceleration_limits=autoware::motion_utils::AccelerationLimits(values);
  }
  void valid(bool abort) {
    status_.is_valid_path=true;
    if(abort){abort_path_=std::make_shared<bpp::LaneChangePath>();toAbortState();}
  }
  void prepareCancel(const bpp::PathWithLaneId & path,bool already_changing=false) {
    status_.is_valid_path=true;status_.lane_change_path.path=path;
    status_.lane_change_path.info.shift_line.start=path.points[already_changing?0:20].point.pose;
    status_.lane_change_path.info.lane_changing_start=status_.lane_change_path.info.shift_line.start;
    status_.lane_change_path.info.lane_changing_end=path.points[80].point.pose;
    status_.lane_change_path.info.lane_changing_end.position.y=4;
    status_.lane_change_path.info.duration.prepare=4;
    status_.lane_change_path.type=bpp::lane_change::PathType::SpatialSpeedProfile;
    lane_change_parameters_->cancel.enable_on_prepare_phase=true;
    lane_change_parameters_->cancel.delta_time=1;
    lane_change_parameters_->trajectory.min_lane_changing_velocity=1;
    lane_change_parameters_->trajectory.max_longitudinal_acc=2;
    lane_change_parameters_->trajectory.min_longitudinal_acc=-4;
    lane_change_parameters_->safety.collision_check.prediction_time_resolution=.5;
    lane_change_parameters_->safety.collision_check.th_yaw_diff=1.57;
    auto & rss=lane_change_parameters_->safety.rss_params;
    rss.front_vehicle_deceleration=-6;rss.rear_vehicle_deceleration=-6;
    rss.rear_vehicle_reaction_time=.5;rss.longitudinal_distance_min_threshold=3;
    rss.lateral_distance_max_threshold=.5;rss.extended_polygon_policy="rectangle";
    lane_change_parameters_->safety.rss_params_for_abort=rss;
    lanelet::LineString3d left(11,{lanelet::Point3d(1,-20,2,0),lanelet::Point3d(2,40,2,0)});
    lanelet::LineString3d right(12,{lanelet::Point3d(3,-20,-2,0),lanelet::Point3d(4,40,-2,0)});
    lanelet::Lanelet lane(42,left,right);
    common_data_ptr_->lanes_ptr->current={lane};
    common_data_ptr_->lanes_polygon_ptr->current=lane.polygon2d().basicPolygon();
  }
};
class OutputGate : public ::testing::Test {
protected:
  ros::NodeHandle node;
  std::shared_ptr<bpp::PlannerData> data;
  bpp::BehaviorModuleOutput input;
  void SetUp() override {
    data=std::make_shared<bpp::PlannerData>();
    auto odom=boost::make_shared<nav_msgs::Odometry>();odom->pose.pose.orientation.w=1;
    data->self_odometry=odom;
    data->route_handler=std::make_shared<autoware::route_handler::RouteHandler>();
    for(int i=0;i<4;++i){bpp::PathPointWithLaneId p;p.point.pose.position.x=i;
      p.point.pose.orientation.w=1;p.point.longitudinal_velocity_mps=10;input.path.points.push_back(p);}
    ros::param::set("/planning/lc_strict_maneuver_safety",false);
  }
  void TearDown() override {for(const auto * key:{"lc_strict_maneuver_safety","lc_fresh_commit_input","lc_continuous_bounded_output","lc_partial_acceleration_candidates","lc_measured_motion_diagnostic_only","lc_skip_unrequested_avoidance_search"})ros::param::set(std::string("/planning/")+key,false);}
  void ready(OutputProbe & p){p.setData(data);p.setPreviousModuleOutput(input);}
  void normalFollowingInput(){
    input.path.points.clear();
    for(int i=0;i<=100;++i){bpp::PathPointWithLaneId p;p.point.pose.position.x=i;
      p.point.pose.orientation.w=1;p.point.longitudinal_velocity_mps=10;p.lane_ids={42};input.path.points.push_back(p);}
    input.reference_path=input.path;
    data->parameters.min_acc=-4;data->parameters.max_acc=2;data->parameters.max_vel=40;
    data->parameters.ego_nearest_dist_threshold=3;data->parameters.ego_nearest_yaw_threshold=1;
    data->parameters.vehicle_info=autoware::vehicle_info_utils::createVehicleInfo(.35,.2,3,1.651,.845,.79,.1195,.1195,1.7,.6);
    auto odom=boost::make_shared<nav_msgs::Odometry>();odom->pose.pose.orientation.w=1;odom->twist.twist.linear.x=5;
    data->self_odometry=odom;
    data->self_acceleration=boost::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
    data->dynamic_object=boost::make_shared<autoware_perception_msgs::PredictedObjects>();
  }
  void assertFailure(const std::function<bpp::BehaviorModuleOutput()> & produce,
                     const std::string & reason){
    std::string event;
    auto sub=node.subscribe<std_msgs::String>("/planning/debug/lane_change_safety_failure",10,
      [&](const std_msgs::String::ConstPtr & msg){event=msg->data;});
    for(int i=0;i<40 && event.empty();++i){
      auto output=produce();
      ASSERT_EQ(output.path.points.size(),input.path.points.size());
      for(const auto & p:output.path.points)EXPECT_DOUBLE_EQ(p.point.longitudinal_velocity_mps,0);
      ros::spinOnce();ros::WallDuration(.025).sleep();
    }
    ASSERT_FALSE(event.empty());EXPECT_NE(event.find("no_safe_maneuver"),std::string::npos);
    EXPECT_NE(event.find(reason),std::string::npos);
    EXPECT_NE(event.find("00000000000000000000000000000000"),std::string::npos);
  }
};
TEST_F(OutputGate, ActualGenerateOutputInvalidEarlyReturnHonorsOptIn){
  OutputProbe p;ready(p);
  EXPECT_DOUBLE_EQ(p.generateOutput().path.points.front().point.longitudinal_velocity_mps,10);
  ros::param::set("/planning/lc_strict_maneuver_safety",true);
  assertFailure([&]{return p.generateOutput();},"invalid_active_candidate");
}
TEST_F(OutputGate, ActualGenerateOutputInvalidAbortDoesNotBypassGate){
  OutputProbe p;ready(p);p.toAbortState();
  ros::param::set("/planning/lc_strict_maneuver_safety",true);
  assertFailure([&]{return p.generateOutput();},"invalid_active_candidate");
}
TEST_F(OutputGate, FinalNormalAndAbortOutputsRejectMissingFreshInputs){
  ros::param::set("/planning/lc_strict_maneuver_safety",true);
  for(bool abort:{false,true}){
    OutputProbe p;ready(p);p.valid(abort);
    assertFailure([&]{auto output=input;p.enforce_strict_maneuver_safety(output);return output;},
      "missing_motion_or_objects");
  }
}
TEST_F(OutputGate, ValidatedPrepareCancelPublishesExactUpstreamNotPredictionSpeeds){
  normalFollowingInput();OutputProbe p;ready(p);p.prepareCancel(input.path);
  ros::param::set("/planning/lc_strict_maneuver_safety",true);
  auto output=input;for(auto & point:output.path.points)point.point.pose.position.y=4;
  p.enforce_strict_maneuver_safety(output);
  EXPECT_TRUE(p.isCancelState());
  EXPECT_EQ(output.path,input.path);EXPECT_EQ(output.reference_path,input.reference_path);
  EXPECT_DOUBLE_EQ(output.path.points.front().point.longitudinal_velocity_mps,10);
}
TEST_F(OutputGate, AlreadyChangingDoesNotTakeCurrentLaneCancelAlternative){
  normalFollowingInput();OutputProbe p;ready(p);p.prepareCancel(input.path,true);
  ros::param::set("/planning/lc_strict_maneuver_safety",true);
  auto output=input;for(auto & point:output.path.points)point.point.pose.position.y=4;
  p.enforce_strict_maneuver_safety(output);
  EXPECT_FALSE(p.isCancelState());
  EXPECT_DOUBLE_EQ(output.path.points.front().point.longitudinal_velocity_mps,0);
}
TEST_F(OutputGate, OncomingRawOverlapRejectsCurrentLaneCancel){
  normalFollowingInput();
  auto objects=boost::make_shared<autoware_perception_msgs::PredictedObjects>();
  autoware_perception_msgs::PredictedObject object;
  object.shape.dimensions.x=4;object.shape.dimensions.y=2;object.shape.dimensions.z=1.5;
  object.kinematics.initial_pose_with_covariance.pose.position.x=10;
  object.kinematics.initial_pose_with_covariance.pose.orientation.z=1;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w=0;
  autoware_perception_msgs::PredictedPath prediction;prediction.confidence=1;
  prediction.time_step.sec=0;prediction.time_step.nanosec=100000000;
  for(int i=0;i<=30;++i)prediction.path.push_back(object.kinematics.initial_pose_with_covariance.pose);
  object.kinematics.predicted_paths={prediction};objects->objects={object};data->dynamic_object=objects;
  OutputProbe p;ready(p);p.prepareCancel(input.path);
  ros::param::set("/planning/lc_strict_maneuver_safety",true);
  auto output=input;for(auto & point:output.path.points)point.point.pose.position.y=4;
  p.enforce_strict_maneuver_safety(output);
  EXPECT_FALSE(p.isCancelState());
  EXPECT_DOUBLE_EQ(output.path.points.front().point.longitudinal_velocity_mps,0);
}
TEST_F(OutputGate, SharedPrepareGeometryAcceptsMeasuredTrackingOffsetWithEnvelope){
  normalFollowingInput();
  auto odom=boost::make_shared<nav_msgs::Odometry>(*data->self_odometry);
  odom->pose.pose.position.y=.12;
  odom->pose.pose.orientation.z=std::sin(.01/2);
  odom->pose.pose.orientation.w=std::cos(.01/2);data->self_odometry=odom;
  input.path.points[40].point.longitudinal_velocity_mps=0;
  OutputProbe p;ready(p);p.prepareCancel(input.path);
  ros::param::set("/planning/lc_strict_maneuver_safety",true);
  auto output=input;for(auto & point:output.path.points)point.point.pose.position.y=4;
  p.enforce_strict_maneuver_safety(output);
  EXPECT_TRUE(p.isCancelState());EXPECT_EQ(output.path,input.path);
}
TEST_F(OutputGate, ActualSharedPreparePathJumpStillRejectsCancel){
  normalFollowingInput();OutputProbe p;ready(p);p.prepareCancel(input.path);
  auto shifted=input;for(auto & point:shifted.path.points)point.point.pose.position.y=.2;
  p.setPreviousModuleOutput(shifted);
  ros::param::set("/planning/lc_strict_maneuver_safety",true);
  auto output=input;for(auto & point:output.path.points)point.point.pose.position.y=4;
  p.enforce_strict_maneuver_safety(output);
  EXPECT_FALSE(p.isCancelState());EXPECT_DOUBLE_EQ(output.path.points.front().point.longitudinal_velocity_mps,0);
}
TEST_F(OutputGate, MeasuredPoseEnvelopeEnclosesTranslationAndRotation){
  normalFollowingInput();namespace lc=bpp::utils::lane_change;
  auto ego=data->self_odometry->pose.pose;ego.position.y=.12;
  ego.orientation.z=std::sin(.04/2);ego.orientation.w=std::cos(.04/2);
  const auto & vehicle=data->parameters.vehicle_info;
  const double r=lc::current_lane_cancel_envelope(input.path,input.path,ego,
    input.path.points[20].point.pose,vehicle);
  EXPECT_GT(r,.12);
  const auto expanded=lc::cancel_envelope_vehicle(vehicle,r);
  const auto actual=bpp::utils::path_safety_checker::create_extended_ego_polygon(ego,vehicle,0,0,false);
  const auto envelope=bpp::utils::path_safety_checker::create_extended_ego_polygon(
    input.path.points[0].point.pose,expanded,0,0,false);
  EXPECT_TRUE(boost::geometry::covered_by(actual,envelope));
}
TEST_F(OutputGate, SnapshotRosSerializationRoundTripsExactPreZeroPathAndHeaders){
  normalFollowingInput();namespace lc=bpp::utils::lane_change;
  input.path.header.stamp=ros::Time(1790952957,423678200);
  input.path.header.frame_id="map";input.path.points[10].point.longitudinal_velocity_mps=0;
  auto packet=lc::snapshot_ros_message(input.path);
  std::ostringstream encoded;boost::property_tree::write_json(encoded,packet,false);
  boost::property_tree::ptree parsed;std::istringstream bytes(encoded.str());boost::property_tree::read_json(bytes,parsed);
  const auto result=lc::decode_snapshot_ros_message<bpp::PathWithLaneId>(parsed);
  EXPECT_EQ(result,input.path);
  EXPECT_THROW(lc::decode_snapshot_ros_message<nav_msgs::Odometry>(parsed),std::logic_error);
  parsed.put("hex",packet.get<std::string>("hex")+"00");
  EXPECT_THROW(lc::decode_snapshot_ros_message<bpp::PathWithLaneId>(parsed),std::logic_error);
  parsed=packet;auto hex=parsed.get<std::string>("hex");hex[0]='z';parsed.put("hex",hex);
  EXPECT_THROW(lc::decode_snapshot_ros_message<bpp::PathWithLaneId>(parsed),std::logic_error);
}
TEST_F(OutputGate, SnapshotPublicationMatchesFailureAndRetainsPreZeroPath){
  normalFollowingInput();OutputProbe p;ready(p);
  std::string failure,snapshot;
  auto fail_sub=node.subscribe<std_msgs::String>("/planning/debug/lane_change_safety_failure",10,
    [&](const std_msgs::String::ConstPtr & m){failure=m->data;});
  auto snapshot_sub=node.subscribe<std_msgs::String>("/planning/debug/lane_change_safety_snapshot",1,
    [&](const std_msgs::String::ConstPtr & m){snapshot=m->data;});
  ros::WallDuration(.55).sleep();
  for(int i=0;i<40 && (!fail_sub.getNumPublishers() || !snapshot_sub.getNumPublishers());++i){ros::spinOnce();ros::WallDuration(.025).sleep();}
  ASSERT_GT(fail_sub.getNumPublishers(),0u);ASSERT_GT(snapshot_sub.getNumPublishers(),0u);
  ros::param::set("/planning/lc_strict_maneuver_safety",true);
  auto output=input;p.enforce_strict_maneuver_safety(output);
  for(int i=0;i<40 && (failure.empty()||snapshot.empty());++i){ros::spinOnce();ros::WallDuration(.025).sleep();}
  ASSERT_FALSE(failure.empty());ASSERT_FALSE(snapshot.empty());
  boost::property_tree::ptree a,b;std::istringstream fa(failure),sb(snapshot);
  boost::property_tree::read_json(fa,a);boost::property_tree::read_json(sb,b);
  EXPECT_DOUBLE_EQ(a.get<double>("stamp"),b.get<double>("failure_stamp"));
  EXPECT_EQ(b.get<std::string>("schema"),"lc_safety_snapshot_v1");
  EXPECT_EQ(bpp::utils::lane_change::decode_snapshot_ros_message<bpp::PathWithLaneId>(b.get_child("output_path")),input.path);
  EXPECT_EQ(bpp::utils::lane_change::decode_snapshot_ros_message<nav_msgs::Odometry>(b.get_child("odometry")),*data->self_odometry);
  for(const auto & point:output.path.points)EXPECT_DOUBLE_EQ(point.point.longitudinal_velocity_mps,0);
}
TEST_F(OutputGate, TinyFutureObjectsUseAdvancedEgoReferenceButLargerSkewRejects){
  for(const double delta:{.00118,.010,.011}){
    normalFollowingInput();
    auto objects=boost::make_shared<autoware_perception_msgs::PredictedObjects>(*data->dynamic_object);
    objects->header.stamp=data->self_odometry->header.stamp+ros::Duration(delta);data->dynamic_object=objects;
    OutputProbe p;ready(p);p.prepareCancel(input.path);
    ros::param::set("/planning/lc_strict_maneuver_safety",true);
    auto output=input;for(auto & point:output.path.points)point.point.pose.position.y=4;
    p.enforce_strict_maneuver_safety(output);
    EXPECT_EQ(p.isCancelState(),delta<=.010);
    if(delta<=.010)EXPECT_EQ(output.path,input.path);
  }
}
TEST_F(OutputGate, SelectedBrakeCapturePreservesBothPathsAndDoesNotThrottleFailure){
  normalFollowingInput();OutputProbe p;ready(p);
  std::vector<boost::property_tree::ptree> packets;
  auto sub=node.subscribe<std_msgs::String>("/planning/debug/lane_change_safety_snapshot",10,
    [&](const std_msgs::String::ConstPtr & m){boost::property_tree::ptree parsed;
      std::istringstream s(m->data);boost::property_tree::read_json(s,parsed);packets.push_back(parsed);});
  ros::WallDuration(.55).sleep();
  for(int i=0;i<40&&!sub.getNumPublishers();++i){ros::spinOnce();ros::WallDuration(.025).sleep();}
  auto chosen=input.path;for(auto & point:chosen.points)point.point.longitudinal_velocity_mps=2;
  const auto original=input.path;const auto before=ros::WallTime::now();
  p.publishStrictSafetySnapshot("checked_braking_selected",{},input.path,&chosen,ros::Time(10),"unsafe","ineligible","safe");
  RecordProperty("selected_capture_ms",(ros::WallTime::now()-before).toSec()*1000);
  for(int i=0;i<20&&packets.empty();++i){ros::spinOnce();ros::WallDuration(.005).sleep();}
  p.publishStrictSafetySnapshot("no_safe_maneuver",{},input.path,nullptr,ros::Time(11),"unsafe","unsafe","unsafe");
  for(int i=0;i<40&&packets.size()<2;++i){ros::spinOnce();ros::WallDuration(.025).sleep();}
  ASSERT_EQ(packets.size(),2u);
  EXPECT_EQ(packets[0].get<std::string>("stage"),"checked_braking_selected");
  EXPECT_EQ(packets[1].get<std::string>("stage"),"no_safe_maneuver");
  EXPECT_EQ(bpp::utils::lane_change::decode_snapshot_ros_message<bpp::PathWithLaneId>(packets[0].get_child("output_path")),original);
  EXPECT_EQ(bpp::utils::lane_change::decode_snapshot_ros_message<bpp::PathWithLaneId>(packets[0].get_child("selected_path")),chosen);
  EXPECT_EQ(input.path,original);
  RecordProperty("two_captures_with_subscriber_wait_ms",(ros::WallTime::now()-before).toSec()*1000);
}
TEST_F(OutputGate, UnapprovedCaptureIsIndependentAndPreservesAtomicInputs){
  normalFollowingInput();OutputProbe p;ready(p);p.prepareCancel(input.path);
  ros::WallDuration(1.01).sleep();
  std::vector<boost::property_tree::ptree> packets;
  auto sub=node.subscribe<std_msgs::String>("/planning/debug/lane_change_safety_snapshot",10,
    [&](const std_msgs::String::ConstPtr & m){boost::property_tree::ptree parsed;
      std::istringstream s(m->data);boost::property_tree::read_json(s,parsed);packets.push_back(parsed);});
  for(int i=0;i<40&&!sub.getNumPublishers();++i){ros::spinOnce();ros::WallDuration(.025).sleep();}
  const auto original=input.path;
  p.publishStrictSafetySnapshot("unapproved_request_candidate",{},input.path,nullptr,
    ros::Time(21),"native_candidate_complete_before_approval","","");
  p.publishStrictSafetySnapshot("unapproved_request_candidate",{},input.path,nullptr,
    ros::Time(22),"native_candidate_complete_before_approval","","");
  p.publishStrictSafetySnapshot("checked_continuous_acceleration_selected",{},input.path,
    &input.path,ros::Time(23),"safe","","");
  for(int i=0;i<40&&packets.size()<2;++i){ros::spinOnce();ros::WallDuration(.025).sleep();}
  ASSERT_EQ(packets.size(),2u);
  const auto & packet=packets.front();
  EXPECT_EQ(packet.get<std::string>("stage"),"unapproved_request_candidate");
  EXPECT_FALSE(packet.get<bool>("approved"));EXPECT_TRUE(packet.get<bool>("candidate_complete"));
  EXPECT_EQ(packet.get<std::string>("route_context"),"externalroute_join_required");
  EXPECT_EQ(packet.get<std::string>("request_source"),"NORMAL_preference");
  EXPECT_EQ(packet.get<std::string>("request_direction"),"LEFT");
  EXPECT_FALSE(packet.get<bool>("partial_hard_envelope_valid"));
  EXPECT_EQ(bpp::utils::lane_change::decode_snapshot_ros_message<bpp::PathWithLaneId>(packet.get_child("upstream_path")),original);
  EXPECT_EQ(bpp::utils::lane_change::decode_snapshot_ros_message<nav_msgs::Odometry>(packet.get_child("odometry")),*data->self_odometry);
  EXPECT_EQ(input.path,original);
}
TEST_F(OutputGate, TimeRebasePreservesExactHorizonAndStopRejectsMissingCoverage){
  namespace lc=bpp::utils::lane_change;
  lc::SafetyPrediction input;
  for(int i=0;i<=31;++i){
    input.emplace_back();auto & p=input.back();p.time=.1*i;
    p.pose.position.x=std::min(p.time,1.0);p.pose.orientation.w=1;
    p.velocity=p.time<1?1:0;
  }
  const auto result=lc::rebase_observation_prediction(input,.01,3);
  EXPECT_DOUBLE_EQ(result.front().time,0);EXPECT_NEAR(result.front().pose.position.x,.01,1e-8);
  EXPECT_DOUBLE_EQ(result.back().time,3);EXPECT_DOUBLE_EQ(result.back().velocity,0);
  EXPECT_DOUBLE_EQ(result.back().pose.position.x,1);
  input.pop_back();EXPECT_THROW(lc::rebase_observation_prediction(input,.01,3),std::logic_error);
}

class AvoidanceSearchProbe : public bpp::NormalLaneChange {
public:
  explicit AvoidanceSearchProbe(bpp::LaneChangeModuleType type=bpp::LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE)
  :NormalLaneChange(std::make_shared<bpp::LaneChangeParameters>(),type,bpp::Direction::LEFT){}
  bool required=false;mutable unsigned searches=0;
  bool specialRequiredCheck()const override{return required;}
  std::pair<bool,bool> getSafePath(bpp::LaneChangePath & path)const override {
    ++searches;path.path.points.resize(2);return {true,true};
  }
  void seed(bool approved,bool abort=false){transient_is_approved_=approved;
    status_.is_valid_path=true;status_.is_safe=true;status_.lane_change_path.path.points.resize(3);
    selected_output_model_=status_.lane_change_path;terminal_lane_change_path_=status_.lane_change_path;
    if(abort){abort_path_=std::make_shared<bpp::LaneChangePath>(status_.lane_change_path);toAbortState();}}
  bool oldCacheCleared()const{return !selected_output_model_&&!terminal_lane_change_path_;}
};
TEST_F(OutputGate, UnrequestedAvoidanceSearchClearsStaleCandidateAndResumes)
{
  normalFollowingInput();AvoidanceSearchProbe p;p.setData(data);p.setPreviousModuleOutput(input);p.seed(false);
  ros::param::set("/planning/lc_skip_unrequested_avoidance_search",true);
  p.updateLaneChangeStatus();EXPECT_EQ(p.searches,0u);EXPECT_FALSE(p.bpp::LaneChangeBase::isValidPath());
  EXPECT_FALSE(p.isSafe());EXPECT_TRUE(p.getLaneChangeStatus().lane_change_path.path.points.empty());
  EXPECT_TRUE(p.oldCacheCleared());
  p.required=true;p.updateLaneChangeStatus();EXPECT_EQ(p.searches,1u);EXPECT_TRUE(p.bpp::LaneChangeBase::isValidPath());EXPECT_TRUE(p.isSafe());
}
TEST_F(OutputGate, AvoidanceSearchSkipNeverPrunesApprovedAbortNormalOrExternal)
{
  normalFollowingInput();ros::param::set("/planning/lc_skip_unrequested_avoidance_search",true);
  for(const auto type:{bpp::LaneChangeModuleType::NORMAL,bpp::LaneChangeModuleType::EXTERNAL_REQUEST,
      bpp::LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE}) {
    AvoidanceSearchProbe p(type);p.setData(data);p.setPreviousModuleOutput(input);
    p.seed(type==bpp::LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE);
    p.updateLaneChangeStatus();EXPECT_EQ(p.searches,1u);
  }
  AvoidanceSearchProbe aborted;aborted.setData(data);aborted.setPreviousModuleOutput(input);aborted.seed(false,true);
  aborted.updateLaneChangeStatus();EXPECT_EQ(aborted.searches,1u);
  AvoidanceSearchProbe stopped;stopped.setData(data);stopped.setPreviousModuleOutput(input);stopped.seed(false);
  stopped.toStopState();stopped.updateLaneChangeStatus();EXPECT_EQ(stopped.searches,1u);
  AvoidanceSearchProbe cancelled;cancelled.setData(data);cancelled.setPreviousModuleOutput(input);cancelled.seed(false);
  cancelled.toCancelState();cancelled.updateLaneChangeStatus();EXPECT_EQ(cancelled.searches,1u);
  ros::param::set("/planning/lc_skip_unrequested_avoidance_search",false);
  AvoidanceSearchProbe off;off.setData(data);off.setPreviousModuleOutput(input);off.seed(false);
  off.updateLaneChangeStatus();EXPECT_EQ(off.searches,1u);
}

TEST_F(OutputGate, FreshProviderR47ExactEpochThroughModule){
  const char * directory=std::getenv("AJOU_FRESH_COMMIT_FIXTURE_DIR");
  const char * map_file=std::getenv("AJOU_FRESH_COMMIT_MAP_FILE");
  if(!directory||!map_file)GTEST_SKIP()<<"Set exact R47 JSON fixture directory and map file";
  namespace lc=bpp::utils::lane_change;
  boost::property_tree::ptree old,fresh;
  boost::property_tree::read_json(std::string(directory)+"/r47_002_original.json",old);
  boost::property_tree::read_json(std::string(directory)+"/r47_002_retimed.json",fresh);
  autoware_map_msgs::MapProjectorInfo projector;projector.projector_type="LocalCartesianUTM";
  projector.map_origin.latitude=37.2380038082;projector.map_origin.longitude=126.7730819386;
  auto map=autoware::map_loader::load_lanelet2_map(map_file,projector);
  lanelet::utils::overwriteLaneletsCenterlineWithWaypoints(map,5.,false);
  data->route_handler=std::make_shared<autoware::route_handler::RouteHandler>(map);
  data->route_handler->setRoute(lc::decode_snapshot_ros_message<autoware_planning_msgs::LaneletRoute>(old.get_child("fixture_route")));
  auto motion=[](bpp::PlannerData & d,const boost::property_tree::ptree&p){
    d.self_odometry=boost::make_shared<nav_msgs::Odometry>(lc::decode_snapshot_ros_message<nav_msgs::Odometry>(p.get_child("odometry")));
    d.self_acceleration=boost::make_shared<geometry_msgs::AccelWithCovarianceStamped>(lc::decode_snapshot_ros_message<geometry_msgs::AccelWithCovarianceStamped>(p.get_child("acceleration")));
    d.dynamic_object=boost::make_shared<autoware_perception_msgs::PredictedObjects>(lc::decode_snapshot_ros_message<autoware_perception_msgs::PredictedObjects>(p.get_child("objects")));};
  motion(*data,old);
  auto f=[&](const char*k){return old.get<double>(std::string("vehicle.")+k);};
  data->parameters.vehicle_info=autoware::vehicle_info_utils::createVehicleInfo(f("wheel_radius_m"),f("wheel_width_m"),f("wheel_base_m"),f("wheel_tread_m"),f("front_overhang_m"),f("rear_overhang_m"),f("left_overhang_m"),f("right_overhang_m"),f("vehicle_height_m"),f("max_steer_angle_rad"));
  data->parameters.max_vel=old.get<double>("limits.max_vel");data->parameters.max_acc=old.get<double>("limits.common_max_acc");data->parameters.min_acc=old.get<double>("limits.common_min_acc");
  data->parameters.ego_nearest_dist_threshold=old.get<double>("limits.ego_nearest_dist_threshold");data->parameters.ego_nearest_yaw_threshold=old.get<double>("limits.ego_nearest_yaw_threshold");
  input.path=lc::decode_snapshot_ros_message<bpp::PathWithLaneId>(old.get_child("upstream_path"));input.reference_path=input.path;
  OutputProbe p;ready(p);p.configureFreshFixture(old);
  bpp::LaneChangePath candidate;candidate.type=bpp::lane_change::PathType::SpatialSpeedProfile;
  candidate.path=lc::decode_snapshot_ros_message<bpp::PathWithLaneId>(old.get_child("output_path"));
  candidate.info.lane_changing_start=lc::decode_snapshot_ros_message<geometry_msgs::Pose>(old.get_child("candidate.start"));
  candidate.info.lane_changing_end=lc::decode_snapshot_ros_message<geometry_msgs::Pose>(old.get_child("candidate.end"));
  candidate.info.shift_line.start=candidate.info.lane_changing_start;candidate.info.shift_line.end=candidate.info.lane_changing_end;
  candidate.info.duration.prepare=old.get<double>("candidate.duration.prepare");
  auto selected=lc::decode_snapshot_ros_message<bpp::PathWithLaneId>(old.get_child("selected_path"));
  ros::param::set("/planning/lc_measured_motion_diagnostic_only",true);
  ros::param::set("/planning/interpolate_rising_velocity",true);
  std::string reason;ASSERT_TRUE(p.check_strict_maneuver_safety(candidate,selected,reason,true))<<reason;
  const auto compare_world=[&](const bpp::PathWithLaneId & path){
    const auto start=ros::WallTime::now();
    const auto world=p.prepare_admission_world();
    const double prepare_ms=(ros::WallTime::now()-start).toSec()*1000;
    std::string uncached_reason,cached_reason;
    const auto old_start=ros::WallTime::now();
    const bool uncached=p.check_strict_maneuver_safety(candidate,path,uncached_reason,false);
    const double old_ms=(ros::WallTime::now()-old_start).toSec()*1000;
    const auto cached_start=ros::WallTime::now();
    const bool cached=p.check_strict_maneuver_safety(candidate,path,cached_reason,false,false,world.get());
    const double cached_ms=(ros::WallTime::now()-cached_start).toSec()*1000;
    EXPECT_EQ(cached,uncached);EXPECT_EQ(cached_reason,uncached_reason);
    std::cout<<"prepared world differential: prepare_ms="<<prepare_ms<<" uncached_ms="<<old_ms
      <<" cached_ms="<<cached_ms<<" accepted="<<cached<<std::endl;
    auto wrong=*world;wrong.resolution+=.01;
    EXPECT_FALSE(p.check_strict_maneuver_safety(candidate,path,cached_reason,false,false,&wrong));
    EXPECT_EQ(cached_reason,"prepared_safety_world_epoch_mismatch");
    wrong=*world;wrong.raw=nullptr;
    EXPECT_FALSE(p.check_strict_maneuver_safety(candidate,path,cached_reason,false,false,&wrong));
    wrong=*world;std::reverse(wrong.current_ids.begin(),wrong.current_ids.end());
    if(wrong.current_ids!=world->current_ids)
      EXPECT_FALSE(p.check_strict_maneuver_safety(candidate,path,cached_reason,false,false,&wrong));
    wrong=*world;wrong.route_uuid.uuid[0]^=1;
    EXPECT_FALSE(p.check_strict_maneuver_safety(candidate,path,cached_reason,false,false,&wrong));
    wrong=*world;wrong.odom_stamp+=ros::Duration(.001);
    EXPECT_FALSE(p.check_strict_maneuver_safety(candidate,path,cached_reason,false,false,&wrong));
    EXPECT_FALSE(p.check_strict_maneuver_safety(candidate,path,cached_reason,false,true,world.get()));
  };
  compare_world(selected);
  const auto original_odom=data->self_odometry;auto latest=std::make_shared<bpp::PlannerData>(*data);motion(*latest,fresh);
  bpp::fresh_commit::Snapshot snapshot;snapshot.planner_data=latest;snapshot.acquired_at=ros::Time(fresh.get<double>("failure_stamp"));snapshot.acquired_wall=ros::WallTime::now();snapshot.owner=data.get();snapshot.map_identity=map.get();snapshot.route_handler=data->route_handler.get();snapshot.route_uuid=data->route_handler->getRouteUuid();snapshot.route_stamp=data->route_handler->getRouteHeader().stamp;
  bpp::fresh_commit::ScopedProvider provider([&](){return std::optional<bpp::fresh_commit::Snapshot>(snapshot);});
  ASSERT_TRUE(p.acquire_fresh_output_context());EXPECT_EQ(p.privateData().get(),data.get());
  for(const auto * key:{"lc_fresh_commit_input","lc_strict_maneuver_safety","lc_continuous_bounded_output","lc_partial_acceleration_candidates"})ros::param::set(std::string("/planning/")+key,true);
  ASSERT_TRUE(p.acquire_fresh_output_context());EXPECT_EQ(data->self_odometry,original_odom);EXPECT_EQ(p.privateData()->self_odometry,latest->self_odometry);
  EXPECT_FALSE(p.check_strict_maneuver_safety(candidate,selected,reason,true))<<reason;
  // This old publication belongs to the previous ego epoch. Reject its
  // instantaneous initial-speed mismatch before using it as an RSS hypothesis.
  // The fresh actual-v/a bounded witness below remains independently required.
  EXPECT_EQ(reason,"spatial initial speed differs from actual state");
  compare_world(selected);
  const auto pose=latest->self_odometry->pose.pose;
  const auto anchored=lc::anchor_maneuver_at_ego(candidate.path,
    latest->findEgoSegmentIndex(candidate.path.points),pose.position,true);
  const auto begin=anchored.second;
  const auto end=autoware::motion_utils::findNearestIndex(anchored.first.points,candidate.info.lane_changing_end.position);
  const auto start=autoware::motion_utils::findNearestIndex(anchored.first.points,candidate.info.lane_changing_start.position);
  const auto current=lc::decode_snapshot_ros_message<bpp::PathWithLaneId>(old.get_child("current_speed_reference"));
  const auto target=lc::decode_snapshot_ros_message<bpp::PathWithLaneId>(old.get_child("target_speed_reference"));
  const auto caps=lc::make_hard_speed_envelope(anchored.first,current,target,input.path,begin,end,pose.position,
    data->parameters.max_vel,old.get<double>("partial_hard_lateral_limit"),start,true,&candidate.path,begin);
  std::vector<double> arc{0};for(size_t i=begin+1;i<anchored.first.points.size();++i){
    const auto&a=anchored.first.points[i-1].point.pose.position;const auto&b=anchored.first.points[i].point.pose.position;
    arc.push_back(arc.back()+std::hypot(b.x-a.x,b.y-a.y));}
  const double horizon=3-(latest->self_odometry->header.stamp-latest->dynamic_object->header.stamp).toSec();
  const auto brake=bounded_speed_grid::make(arc,caps,latest->self_odometry->twist.twist.linear.x,
    latest->self_acceleration->accel.accel.linear.x,-4,horizon,-4,
    old.get<double>("limits.smoother_min_jerk"),old.get<double>("limits.smoother_max_jerk"),
    [&](double v){return p.limit(v,std::min(data->parameters.max_acc,old.get<double>("limits.lc_max_acc")));});
  auto corrected=anchored.first;for(size_t i=0;i<brake.speeds.size();++i)corrected.points[begin+i].point.longitudinal_velocity_mps=brake.speeds[i];
  ASSERT_TRUE(p.check_strict_maneuver_safety(candidate,corrected,reason,true))<<reason;
  compare_world(corrected);
  // Recorded-input complete candidate family: one immutable world for all
  // generated proposals, independently compared with the uncached wrapper.
  const auto prepared_start=ros::WallTime::now();
  const auto family_world=p.prepare_admission_world();
  const double prepare_ms=(ros::WallTime::now()-prepared_start).toSec()*1000;
  const auto accel_limit=[&](double v){return p.limit(v,std::min(data->parameters.max_acc,old.get<double>("limits.lc_max_acc")));};
  const auto targets=bpp::lane_change::rss_positive_grid::augment(bounded_speed_grid::acceleration_targets(
    accel_limit(latest->self_odometry->twist.twist.linear.x),-4),
    accel_limit(latest->self_odometry->twist.twist.linear.x),true);
  size_t generated=0;double uncached_ms=0,cached_ms=0;
  for(double target_acc:targets){
    try{
      const auto proposal=bounded_speed_grid::make(arc,caps,latest->self_odometry->twist.twist.linear.x,
        latest->self_acceleration->accel.accel.linear.x,target_acc,horizon,-4,
        old.get<double>("limits.smoother_min_jerk"),old.get<double>("limits.smoother_max_jerk"),accel_limit);
      auto path=anchored.first;
      for(size_t i=0;i<proposal.speeds.size();++i)path.points[begin+i].point.longitudinal_velocity_mps=proposal.speeds[i];
      std::string a,b;auto t=ros::WallTime::now();
      const bool ua=p.check_strict_maneuver_safety(candidate,path,a,false);
      uncached_ms+=(ros::WallTime::now()-t).toSec()*1000;t=ros::WallTime::now();
      const bool ca=p.check_strict_maneuver_safety(candidate,path,b,false,false,family_world.get());
      cached_ms+=(ros::WallTime::now()-t).toSec()*1000;
      EXPECT_EQ(ua,ca)<<target_acc;EXPECT_EQ(a,b)<<target_acc;++generated;
    }catch(const std::logic_error&){/* Same scalar generator, no changed safety path. */}
  }
  EXPECT_GT(generated,0U);
  std::cout<<"prepared family differential: targets="<<targets.size()<<" generated="<<generated
    <<" prepare_ms="<<prepare_ms<<" uncached_ms="<<uncached_ms<<" cached_ms="<<cached_ms<<std::endl;
  snapshot.map_identity=nullptr;
  EXPECT_FALSE(p.acquire_fresh_output_context());EXPECT_FALSE(p.privateData()->dynamic_object);
  EXPECT_FALSE(p.isApprovedPathSafe().is_safe);EXPECT_EQ(data->self_odometry,original_odom);
}


TEST_F(OutputGate, UtilityRequestRefreshIsLocalAndUsesActualProviderEpoch){
  const char * directory=std::getenv("AJOU_FRESH_COMMIT_FIXTURE_DIR");
  const char * map_file=std::getenv("AJOU_FRESH_COMMIT_MAP_FILE");
  if(!directory||!map_file)GTEST_SKIP()<<"Set exact R47 JSON fixture directory and map file";
  namespace lc=bpp::utils::lane_change;
  boost::property_tree::ptree old,fresh;
  boost::property_tree::read_json(std::string(directory)+"/r47_002_original.json",old);
  boost::property_tree::read_json(std::string(directory)+"/r47_002_retimed.json",fresh);
  autoware_map_msgs::MapProjectorInfo projector;projector.projector_type="LocalCartesianUTM";
  projector.map_origin.latitude=37.2380038082;projector.map_origin.longitude=126.7730819386;
  auto map=autoware::map_loader::load_lanelet2_map(map_file,projector);
  lanelet::utils::overwriteLaneletsCenterlineWithWaypoints(map,5.,false);
  data->route_handler=std::make_shared<autoware::route_handler::RouteHandler>(map);
  data->route_handler->setRoute(lc::decode_snapshot_ros_message<autoware_planning_msgs::LaneletRoute>(old.get_child("fixture_route")));
  auto motion=[](bpp::PlannerData & d,const boost::property_tree::ptree&p){
    d.self_odometry=boost::make_shared<nav_msgs::Odometry>(lc::decode_snapshot_ros_message<nav_msgs::Odometry>(p.get_child("odometry")));
    d.self_acceleration=boost::make_shared<geometry_msgs::AccelWithCovarianceStamped>(lc::decode_snapshot_ros_message<geometry_msgs::AccelWithCovarianceStamped>(p.get_child("acceleration")));
    d.dynamic_object=boost::make_shared<autoware_perception_msgs::PredictedObjects>(lc::decode_snapshot_ros_message<autoware_perception_msgs::PredictedObjects>(p.get_child("objects")));};
  motion(*data,old);
  auto f=[&](const char*k){return old.get<double>(std::string("vehicle.")+k);};
  data->parameters.vehicle_info=autoware::vehicle_info_utils::createVehicleInfo(f("wheel_radius_m"),f("wheel_width_m"),f("wheel_base_m"),f("wheel_tread_m"),f("front_overhang_m"),f("rear_overhang_m"),f("left_overhang_m"),f("right_overhang_m"),f("vehicle_height_m"),f("max_steer_angle_rad"));
  data->parameters.max_vel=old.get<double>("limits.max_vel");data->parameters.max_acc=old.get<double>("limits.common_max_acc");data->parameters.min_acc=old.get<double>("limits.common_min_acc");
  data->parameters.ego_nearest_dist_threshold=old.get<double>("limits.ego_nearest_dist_threshold");data->parameters.ego_nearest_yaw_threshold=old.get<double>("limits.ego_nearest_yaw_threshold");
  input.path=lc::decode_snapshot_ros_message<bpp::PathWithLaneId>(old.get_child("upstream_path"));input.reference_path=input.path;
  OutputProbe p;ready(p);p.configureFreshFixture(old);

  p.seedUtilityNativeGuards();
  const auto saved_now=ros::Time::now();
  const double base=data->self_odometry->header.stamp.toSec();
  auto old_odom=boost::make_shared<nav_msgs::Odometry>(*data->self_odometry);
  auto raw=boost::make_shared<autoware_perception_msgs::PredictedObjects>(*data->dynamic_object);
  auto old_acc=boost::make_shared<geometry_msgs::AccelWithCovarianceStamped>(*data->self_acceleration);
  raw->header.stamp.fromSec(base+.0010797);
  data->self_odometry=old_odom;data->self_acceleration=old_acc;data->dynamic_object=raw;
  p.setData(data);p.seedUtilityNativeGuards();
  auto latest=std::make_shared<bpp::PlannerData>(*data);
  auto new_odom=boost::make_shared<nav_msgs::Odometry>(*old_odom);
  auto new_acc=boost::make_shared<geometry_msgs::AccelWithCovarianceStamped>(*old_acc);
  new_odom->header.stamp.fromSec(base+.02);new_acc->header.stamp=new_odom->header.stamp;
  latest->self_odometry=new_odom;latest->self_acceleration=new_acc;
  ros::Time now;now.fromSec(base+.14);ros::Time::setNow(now);
  for(const char* key:{"lc_candidate_reason_audit","lc_defer_unhelpful_preferred_request",
      "lc_strict_maneuver_safety","lc_continuous_bounded_output","lc_partial_acceleration_candidates","lc_fresh_commit_input"})
    ros::param::set(std::string("/planning/")+key,true);
  ros::param::set("/planning/lc_defer_pre_adjacency_normal",false);
  ros::param::set("/planning/lc_path_hazard_preferred_request",false);
  ros::param::set("/planning/scenario_planning/velocity_smoother/normal/min_acc",-4.);
  ros::param::set("/planning/scenario_planning/velocity_smoother/normal/min_jerk",-10.);
  boost::property_tree::ptree event;
  auto sub=node.subscribe<std_msgs::String>("/planning/debug/lc_preference_decision",10,
    [&](const std_msgs::String::ConstPtr & msg){std::istringstream in(msg->data);boost::property_tree::read_json(in,event);});
  const auto original_common=p.privateCommon();const auto original_path=p.utilityStatusPath();
  const auto original_native=original_common->transient_data;
  (void)p.isLaneChangeRequired(); // Advertise and exercise missing-provider fallback.
  for(int i=0;i<30;++i){ros::spinOnce();ros::WallDuration(.01).sleep();}
  bpp::fresh_commit::Snapshot snapshot;snapshot.planner_data=latest;snapshot.acquired_at=now;
  snapshot.owner=this;snapshot.route_handler=data->route_handler.get();
  snapshot.map_identity=map.get();snapshot.route_message_identity=&old; // Node token != handler's copied route pointer.
  snapshot.route_uuid=data->route_handler->getRouteUuid();snapshot.route_stamp=data->route_handler->getRouteHeader().stamp;
  unsigned calls=0;
  {
    bpp::fresh_commit::ScopedProvider provider([&](){++calls;return std::optional<bpp::fresh_commit::Snapshot>(snapshot);});
    event.clear();(void)p.isLaneChangeRequired();
    for(int i=0;i<30&&event.empty();++i){ros::spinOnce();ros::WallDuration(.01).sleep();}
    ASSERT_EQ(calls,1u)<<event.get<std::string>("reason","missing")<<":"<<event.get<std::string>("utility","missing");ASSERT_FALSE(event.empty());
    EXPECT_TRUE(event.get<bool>("input_refreshed"));EXPECT_EQ(event.get<uint64_t>("input_cycle"),provider.cycle());
    EXPECT_TRUE(event.get<bool>("predicate_evaluated"));
    EXPECT_FALSE(event.get<bool>("requested"));EXPECT_EQ(event.get<std::string>("utility"),"deferred");
    EXPECT_DOUBLE_EQ(event.get<double>("odom"),new_odom->header.stamp.toSec());
    EXPECT_NE(event.get<std::string>("utility"),"input_epoch_invalid");
    EXPECT_NE(event.get<std::string>("utility"),"utility_fresh_identity_unavailable");
    EXPECT_EQ(p.privateData().get(),data.get());EXPECT_EQ(p.privateCommon(),original_common);
    EXPECT_EQ(original_common->self_odometry_ptr,old_odom);EXPECT_EQ(data->dynamic_object,raw);
    EXPECT_EQ(p.utilityStatusPath(),original_path);
    EXPECT_DOUBLE_EQ(original_common->transient_data.dist_to_terminal_end,original_native.dist_to_terminal_end);
    // A different fresh pose must drive the containing-lane test, not the old pose.
    new_odom->pose.pose.position.x+=10000.;event.clear();(void)p.isLaneChangeRequired();
    for(int i=0;i<30&&event.empty();++i){ros::spinOnce();ros::WallDuration(.01).sleep();}
    EXPECT_EQ(calls,2u);EXPECT_EQ(event.get<std::string>("utility"),"ego_lane_not_unique_or_target_empty");
    snapshot.map_identity=nullptr;event.clear();(void)p.isLaneChangeRequired();
    for(int i=0;i<30&&event.empty();++i){ros::spinOnce();ros::WallDuration(.01).sleep();}
    EXPECT_EQ(calls,3u);EXPECT_EQ(event.get<std::string>("utility"),"utility_fresh_identity_unavailable");
  }
  for(const char* key:{"lc_candidate_reason_audit","lc_defer_unhelpful_preferred_request"})ros::param::set(std::string("/planning/")+key,false);
  ros::Time::setNow(saved_now);
}
TEST_F(OutputGate, AdmissionDiagnosticIsRateLimitedAndDoesNotPublishFailureOrMutatePath){
  normalFollowingInput();OutputProbe p;ready(p);
  std::vector<boost::property_tree::ptree> packets;unsigned failures=0;
  auto sub=node.subscribe<std_msgs::String>("/planning/debug/lane_change_safety_snapshot",10,
    [&](const std_msgs::String::ConstPtr&m){boost::property_tree::ptree parsed;
      std::istringstream s(m->data);boost::property_tree::read_json(s,parsed);
      if(parsed.get<std::string>("stage")=="reachable_admission_rejected")packets.push_back(parsed);});
  auto failure=node.subscribe<std_msgs::String>("/planning/debug/lane_change_safety_failure",10,
    [&](const std_msgs::String::ConstPtr&){++failures;});
  for(int i=0;i<40&&!sub.getNumPublishers();++i){ros::spinOnce();ros::WallDuration(.025).sleep();}
  ros::spinOnce();failures=0;
  const auto original=input.path;
  bpp::LaneChangePath candidate;candidate.path=original;
  for(int i=0;i<2;++i)p.publishStrictSafetySnapshot("reachable_admission_rejected",candidate,
    original,nullptr,ros::Time(20+i),"speed reference does not cover candidate source=current",
    "not_an_active_maneuver","not_evaluated");
  for(int i=0;i<20;++i){ros::spinOnce();ros::WallDuration(.005).sleep();}
  ASSERT_EQ(packets.size(),1u);EXPECT_EQ(failures,0u);
  EXPECT_EQ(packets[0].get<std::string>("continue_reason"),"speed reference does not cover candidate source=current");
  EXPECT_EQ(bpp::utils::lane_change::decode_snapshot_ros_message<bpp::PathWithLaneId>(packets[0].get_child("output_path")),original);
  EXPECT_EQ(candidate.path,original);EXPECT_EQ(input.path,original);
}
}
int main(int argc,char **argv){
  // This integration test writes opt-in parameters: never use the live master.
  const char * master=std::getenv("ROS_MASTER_URI");
  if(!master || std::string(master)!="http://localhost:11339")return 2;
  ros::init(argc,argv,"lane_change_output_gate_test");
  ::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
