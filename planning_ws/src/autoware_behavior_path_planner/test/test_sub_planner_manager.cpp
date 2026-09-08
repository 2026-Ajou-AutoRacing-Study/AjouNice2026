#include <autoware/behavior_path_planner/sub_planner_manager.hpp>
#include <autoware/behavior_path_planner/source_planner_pipeline.hpp>
#include <autoware/behavior_path_planner_common/utils/root_reference_utils.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <gtest/gtest.h>

namespace bpp = autoware::behavior_path_planner;
namespace {
struct State {
  bool request{true}, waiting{false}, fail{false}, success{false};
  int runs{0}, exits{0}, posts{0};
};
class Module : public bpp::SceneModuleInterface {
public:
  Module(std::string name, std::shared_ptr<State> state, double delta)
  : SceneModuleInterface(name, {}, {}, nullptr, bpp::ModuleStatus::RUNNING),
    state_(state), delta_(delta) {}
protected:
  bool isExecutionRequested() const override { return state_->request; }
  bool isExecutionReady() const override { return true; }
  bool canTransitSuccessState() override { return state_->success; }
  bool canTransitFailureState() override { return state_->fail; }
  void updateData() override {
    if (state_->waiting) waitApproval(); else clearWaitingApproval();
  }
  void postProcess() override { ++state_->posts; }
  void processOnExit() override { ++state_->exits; }
  bpp::CandidateOutput planCandidate() const override { return {}; }
  bpp::BehaviorModuleOutput planWaitingApproval() override { return plan(); }
  bpp::BehaviorModuleOutput plan() override {
    ++state_->runs;
    auto output = getPreviousModuleOutput();
    output.path.points.front().point.pose.position.x += delta_;
    return output;
  }
private:
  std::shared_ptr<State> state_;
  double delta_;
};
class Manager : public bpp::SceneModuleManagerInterface {
public:
  Manager(std::string name, double delta, bool cooperative = true)
  : SceneModuleManagerInterface(name), delta_(delta), cooperative_(cooperative) {}
  std::shared_ptr<State> state = std::make_shared<State>();
  bool isSimultaneousExecutableAsApprovedModule() const override { return cooperative_; }
  bool isSimultaneousExecutableAsCandidateModule() const override { return cooperative_; }
protected:
  std::unique_ptr<bpp::SceneModuleCore> createNewSceneModuleInstance() override {
    return std::make_unique<Module>(name(), state, delta_);
  }
private:
  double delta_;
  bool cooperative_;
};
class Arbitration : public testing::Test {
protected:
  void SetUp() override { ros::Time::init(); input.valid_output.path.points.resize(1); }
  std::shared_ptr<std::optional<lanelet::ConstLanelet>> lane =
    std::make_shared<std::optional<lanelet::ConstLanelet>>();
  std::unordered_map<std::string, double> timing;
  bpp::ModuleUpdateInfo debug;
  bpp::SubPlannerManager slot{lane, timing, debug};
  std::shared_ptr<bpp::PlannerData> data = std::make_shared<bpp::PlannerData>();
  bpp::SlotOutput input;
  std::shared_ptr<Manager> add(std::string name, double delta, bool cooperative = true) {
    auto manager = std::make_shared<Manager>(name, delta, cooperative);
    manager->setData(data);
    timing[name] = 0;
    slot.addSceneModuleManager(manager);
    return manager;
  }
  static double x(const bpp::SlotOutput & output) {
    return output.valid_output.path.points.front().point.pose.position.x;
  }
};
TEST_F(Arbitration, CooperativeApprovedModulesComposeInPriorityOrder) {
  const auto a = add("a", 1), b = add("b", 10);
  EXPECT_DOUBLE_EQ(x(slot.propagateFull(data, input)), 11);
  ASSERT_EQ(slot.approved_modules().size(), 2U);
  EXPECT_EQ(slot.approved_modules()[0]->name(), "a");
  EXPECT_EQ(slot.approved_modules()[1]->name(), "b");
  EXPECT_DOUBLE_EQ(x(slot.propagateFull(data, input)), 11);
  EXPECT_TRUE(slot.approved_modules()[0]->isOutputPathLocked());
  EXPECT_FALSE(slot.approved_modules()[1]->isOutputPathLocked());
  EXPECT_EQ(a->state->runs, a->state->posts);
  EXPECT_EQ(b->state->runs, b->state->posts);
}
TEST_F(Arbitration, ExclusiveCandidatePreventsLowerPriorityLaunch) {
  const auto a = add("a", 1, false), b = add("b", 10);
  a->state->waiting = true;
  const auto output = slot.propagateFull(data, input);
  EXPECT_TRUE(output.is_upstream_candidate_exclusive);
  EXPECT_DOUBLE_EQ(x(output), 1);
  ASSERT_EQ(slot.candidate_modules().size(), 1U);
  EXPECT_TRUE(slot.approved_modules().empty());
  EXPECT_EQ(b->state->runs, 0);
}
TEST_F(Arbitration, UpstreamExclusiveRunsApprovedButDoesNotLaunchNewCandidate) {
  const auto a = add("a", 1), b = add("b", 10);
  b->state->request = false;
  slot.propagateFull(data, input);
  b->state->request = true;
  input.is_upstream_candidate_exclusive = true;
  const auto output = slot.propagateWithExclusiveCandidate(data, input);
  EXPECT_TRUE(output.is_upstream_candidate_exclusive);
  EXPECT_DOUBLE_EQ(x(output), 1);
  EXPECT_EQ(b->state->runs, 0);
}
TEST_F(Arbitration, FailedApprovedDiscardsItsAndFollowingOutputs) {
  const auto a = add("a", 1), b = add("b", 10);
  slot.propagateFull(data, input);
  // Upstream arbitration retires the lower-priority initial candidate when
  // promoting a, then creates b as an approved module. Count failure-cycle
  // exits independently of that legitimate candidate retirement.
  const auto a_exits = a->state->exits, b_exits = b->state->exits;
  a->state->fail = true;
  const auto output = slot.propagateFull(data, input);
  EXPECT_TRUE(output.is_upstream_failed_approved);
  EXPECT_DOUBLE_EQ(x(output), 0);
  EXPECT_TRUE(slot.approved_modules().empty());
  EXPECT_EQ(a->state->exits - a_exits, 1);
  EXPECT_EQ(b->state->exits - b_exits, 1);
}
TEST_F(Arbitration, UpstreamFailureClearsOwnedModules) {
  const auto a = add("a", 1);
  slot.propagateFull(data, input);
  slot.propagateWithFailedApproved();
  EXPECT_TRUE(slot.approved_modules().empty());
  EXPECT_TRUE(slot.candidate_modules().empty());
  EXPECT_EQ(a->state->exits, 1);
}
TEST_F(Arbitration, SuccessfulApprovedKeepsFinalOutputThenExpires) {
  const auto a = add("a", 1);
  slot.propagateFull(data, input);
  a->state->success = true;
  const auto output = slot.propagateFull(data, input);
  EXPECT_DOUBLE_EQ(x(output), 1);
  EXPECT_FALSE(output.is_upstream_failed_approved);
  EXPECT_TRUE(slot.approved_modules().empty());
  EXPECT_EQ(a->state->exits, 1);
}
TEST_F(Arbitration, ApprovedReturningToWaitingMovesBackToCandidate) {
  const auto a = add("a", 1);
  slot.propagateFull(data, input);
  a->state->waiting = true;
  const auto output = slot.propagateFull(data, input);
  EXPECT_TRUE(output.is_upstream_waiting_approved);
  EXPECT_TRUE(slot.approved_modules().empty());
  ASSERT_EQ(slot.candidate_modules().size(), 1U);
  EXPECT_EQ(slot.candidate_modules().front()->name(), "a");
  EXPECT_EQ(a->state->exits, 0);
}
TEST_F(Arbitration, UpstreamWaitingClearsCandidateBeforeExclusivePropagation) {
  const auto a = add("a", 1);
  a->state->waiting = true;
  slot.propagateFull(data, input);
  ASSERT_EQ(slot.candidate_modules().size(), 1U);
  input.is_upstream_waiting_approved = true;
  input.is_upstream_candidate_exclusive = true;
  const auto output = slot.propagateWithWaitingApproved(data, input);
  EXPECT_TRUE(output.is_upstream_waiting_approved);
  EXPECT_TRUE(output.is_upstream_candidate_exclusive);
  EXPECT_TRUE(slot.candidate_modules().empty());
  EXPECT_DOUBLE_EQ(x(output), 0);
  EXPECT_EQ(a->state->exits, 1);
}
TEST(RootReference, UniqueLaneletIdsPreserveFirstOccurrenceOrder) {
  autoware_internal_planning_msgs::PathWithLaneId path;
  path.points.resize(3);
  path.points[0].lane_ids = {3, 1};
  path.points[1].lane_ids = {1, 2};
  path.points[2].lane_ids = {3, 4};
  EXPECT_EQ(bpp::utils::get_lanelet_id_from_path(path), (std::vector<lanelet::Id>{3, 1, 2, 4}));
}
TEST(RootReference, FreespaceBoundsExtendForwardAndReverseEndpoints) {
  for (const bool forward : {true, false}) {
    autoware_internal_planning_msgs::PathWithLaneId path;
    path.points.resize(3);
    for (size_t i = 0; i < 3; ++i) {
      path.points[i].point.pose.position.x = forward ? 3.0 * i : 6.0 - 3.0 * i;
      path.points[i].point.pose.orientation.w = 1.0;
    }
    bpp::utils::generateDrivableArea(path, 4.0, 2.5, forward);
    ASSERT_EQ(path.left_bound.size(), 5U);
    ASSERT_EQ(path.right_bound.size(), 5U);
    EXPECT_DOUBLE_EQ(path.left_bound.front().x, forward ? -4.0 : 10.0);
    EXPECT_DOUBLE_EQ(path.left_bound.back().x, forward ? 10.0 : -4.0);
    for (size_t i = 0; i < path.left_bound.size(); ++i) {
      EXPECT_DOUBLE_EQ(path.left_bound[i].x, path.right_bound[i].x);
      EXPECT_DOUBLE_EQ(path.left_bound[i].y, 2.5);
      EXPECT_DOUBLE_EQ(path.right_bound[i].y, -2.5);
    }
  }
}
TEST(RootReference, EmptyFreespacePathDoesNotInventBounds) {
  autoware_internal_planning_msgs::PathWithLaneId path;
  bpp::utils::generateDrivableArea(path, 4.0, 2.5, true);
  EXPECT_TRUE(path.left_bound.empty());
  EXPECT_TRUE(path.right_bound.empty());
}
TEST(SourcePipeline, FrozenMapTwoSlotsPropagateFailureWithoutRunningDownstream) {
  if (!ros::isInitialized()) {
    int argc = 0;
    char ** argv = nullptr;
    ros::init(argc, argv, "source_pipeline_test");
  }
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(
    autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map));
  lanelet::ConstLanelets route_lanes;
  autoware_planning_msgs::LaneletRoute route;
  route.header.frame_id = "map";
  for (const auto & lane : map->laneletLayer) {
    const auto line = lane.centerline();
    if (line.size() < 2) continue;
    const auto & first = line.front();
    const auto & last = line.back();
    const double yaw = std::atan2(last.y() - first.y(), last.x() - first.x());
    route.start_pose.position.x = first.x();
    route.start_pose.position.y = first.y();
    route.start_pose.position.z = first.z();
    route.start_pose.orientation.z = std::sin(yaw / 2.0);
    route.start_pose.orientation.w = std::cos(yaw / 2.0);
    route.goal_pose = route.start_pose;
    route.goal_pose.position.x = (first.x() + last.x()) / 2.0;
    route.goal_pose.position.y = (first.y() + last.y()) / 2.0;
    if (route_handler->planPathLaneletsBetweenCheckpoints(route.start_pose, route.goal_pose, &route_lanes))
      break;
  }
  ASSERT_FALSE(route_lanes.empty());
  route_handler->setRouteLanelets(route_lanes);
  route.segments = route_handler->createMapSegments(route_lanes);
  ASSERT_FALSE(route.segments.empty());
  route_handler->setRoute(route);
  ASSERT_TRUE(route_handler->isHandlerReady());
  auto data = std::make_shared<bpp::PlannerData>();
  data->route_handler = route_handler;
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->pose.pose = route.start_pose;
  data->self_odometry = odometry;
  data->parameters.forward_path_length = 30.0;
  data->parameters.backward_path_length = 5.0;
  data->parameters.input_path_interval = 1.0;
  data->parameters.output_path_interval = 1.0;
  data->parameters.ego_nearest_dist_threshold = 3.0;
  data->parameters.ego_nearest_yaw_threshold = 1.57;
  data->parameters.vehicle_length = 4.8;
  auto a = std::make_shared<Manager>("a", 0.0);
  auto b = std::make_shared<Manager>("b", 0.0);
  ros::NodeHandle node("source_pipeline_test");
  bpp::SourcePlannerPipeline pipeline(node, {a, b}, {{"a"}, {"b"}});
  const auto first = pipeline.run(data);
  ASSERT_FALSE(first.path.points.empty());
  EXPECT_FALSE(first.path.left_bound.empty());
  EXPECT_FALSE(first.path.right_bound.empty());
  EXPECT_EQ(pipeline.activeModuleCount(), 2U);
  const int previous_b_runs = b->state->runs, previous_b_exits = b->state->exits;
  a->state->fail = true;
  const auto second = pipeline.run(data);
  EXPECT_FALSE(second.path.points.empty());
  EXPECT_EQ(b->state->runs, previous_b_runs);
  EXPECT_EQ(b->state->exits - previous_b_exits, 1);
  EXPECT_EQ(pipeline.activeModuleCount(), 0U);
}
}  // namespace
