#include "trajectory_validator/planning_status_panel.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <QDockWidget>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QSizePolicy>
#include <QTimer>
#include <QVBoxLayout>

#include <autoware_internal_planning_msgs/PlanningFactor.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz/panel_dock_widget.h>
#include <rviz/visualization_frame.h>
#include <rviz/visualization_manager.h>

namespace trajectory_validator
{
namespace
{
constexpr double kRateWindowSec = 2.0;
constexpr double kSignalTimeoutSec = 1.0;
constexpr double kDecisionTimeoutSec = 1.5;
constexpr std::size_t kMaxDecisionTextLength = 52;
constexpr std::size_t kMaxPlanningFactorDetailLength = 30;
constexpr std::size_t kMaxStopReasonCount = 3;

QLabel * makeCaption(const QString & text)
{
  auto * label = new QLabel(text);
  label->setStyleSheet("QLabel { color: #4b4f54; font: 700 11px 'DejaVu Sans'; }");
  return label;
}

QLabel * makeStatusValue(const QString & initial_text)
{
  auto * label = new QLabel(initial_text);
  label->setAlignment(Qt::AlignCenter);
  label->setWordWrap(true);
  label->setMinimumHeight(42);
  label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  return label;
}

QGroupBox * makeStatusGroup(
  const QString & title, const QString & caption, QLabel ** value_label)
{
  auto * group = new QGroupBox(title);
  auto * layout = new QVBoxLayout(group);
  layout->setContentsMargins(8, 9, 8, 8);
  layout->setSpacing(4);
  layout->addWidget(makeCaption(caption));
  *value_label = makeStatusValue("NO SIGNAL");
  layout->addWidget(*value_label);
  return group;
}

bool contains(const std::string & value, const std::string & token)
{
  return value.find(token) != std::string::npos;
}

bool isSpace(const char value)
{
  return std::isspace(static_cast<unsigned char>(value)) != 0;
}

std::string trimCopy(const std::string & value)
{
  const auto begin = std::find_if_not(value.begin(), value.end(), isSpace);
  if (begin == value.end()) {
    return "";
  }
  const auto end = std::find_if_not(value.rbegin(), value.rend(), isSpace).base();
  return std::string(begin, end);
}

std::string collapseWhitespace(const std::string & value)
{
  std::string output;
  output.reserve(value.size());
  bool previous_was_space = false;
  for (const char character : value) {
    if (isSpace(character)) {
      if (!previous_was_space) {
        output.push_back(' ');
      }
      previous_was_space = true;
      continue;
    }
    output.push_back(character);
    previous_was_space = false;
  }
  return trimCopy(output);
}

std::string firstClause(const std::string & value)
{
  std::size_t end = std::string::npos;
  const std::vector<std::string> separators = {";", ",", "\n", " | "};
  for (const auto & separator : separators) {
    const std::size_t position = value.find(separator);
    if (position != std::string::npos) {
      end = std::min(end, position);
    }
  }
  return trimCopy(end == std::string::npos ? value : value.substr(0, end));
}

std::string abbreviate(const std::string & value, const std::size_t max_length)
{
  const std::string compact = collapseWhitespace(value);
  if (compact.size() <= max_length) {
    return compact;
  }
  if (max_length <= 3) {
    return compact.substr(0, max_length);
  }
  return compact.substr(0, max_length - 3) + "...";
}

std::string compactDecisionText(
  const std::string & value, const std::size_t max_length = kMaxDecisionTextLength)
{
  return abbreviate(firstClause(value), max_length);
}

std::string compactModuleName(std::string module)
{
  const std::size_t slash = module.find_last_of("/.");
  if (slash != std::string::npos && slash + 1 < module.size()) {
    module = module.substr(slash + 1);
  }
  if (contains(module, "behavior_velocity_planner")) {
    return "BVP";
  }
  if (contains(module, "behavior_path_planner")) {
    return "BPP";
  }
  if (contains(module, "static_obstacle_avoidance")) {
    return "Static avoid";
  }
  if (contains(module, "dynamic_obstacle_avoidance")) {
    return "Dynamic avoid";
  }
  if (contains(module, "lane_change")) {
    return "Lane change";
  }
  if (module == "stop_line") {
    return "Stop line";
  }
  if (module == "detection_area") {
    return "Detection area";
  }
  if (module == "traffic_light") {
    return "Traffic light";
  }
  if (module == "virtual_traffic_light") {
    return "Virtual light";
  }
  if (module == "crosswalk") {
    return "Crosswalk";
  }
  if (module == "walkway") {
    return "Walkway";
  }
  if (module == "blind_spot") {
    return "Blind spot";
  }
  if (module == "merge_from_private") {
    return "Private merge";
  }
  if (module == "no_stopping_area") {
    return "No stopping area";
  }
  if (module == "no_drivable_lane") {
    return "No drivable lane";
  }
  if (module == "occlusion_spot") {
    return "Occlusion";
  }
  if (module == "run_out") {
    return "Run out";
  }
  if (module == "speed_bump") {
    return "Speed bump";
  }
  std::replace(module.begin(), module.end(), '_', ' ');
  return abbreviate(module, 24);
}

std::string lowercaseCopy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char character) {return std::tolower(character);});
  return value;
}

std::vector<std::string> split(const std::string & value, const char delimiter)
{
  std::vector<std::string> parts;
  std::stringstream stream(value);
  std::string part;
  while (std::getline(stream, part, delimiter)) {
    parts.push_back(trimCopy(part));
  }
  return parts;
}

bool isInactiveMotionDetail(const std::string & detail)
{
  const std::string lower = lowercaseCopy(collapseWhitespace(detail));
  if (lower.empty() || lower == "no active modules" || contains(lower, "no stop line on path") ||
      contains(lower, "no regulatory area on path") ||
      contains(lower, "no regulatory element on path") ||
      contains(lower, "no walkway on path") ||
      contains(lower, "no turn_direction lanelet on path") ||
      contains(lower, "no private-to-public transition on path") ||
      contains(lower, "no tagged lanelet on path") ||
      contains(lower, "no speed bump on path")) {
    return true;
  }

  const auto entries = split(lower, ',');
  return !entries.empty() && std::all_of(
    entries.begin(), entries.end(), [](const std::string & entry) {
      return contains(entry, "clear") || contains(entry, "missing_sim_pass");
    });
}

std::string summarizeMotionStatus(const std::string & status)
{
  for (const auto & clause : split(status, ';')) {
    const std::size_t separator = clause.find(':');
    if (separator == std::string::npos) {
      if (!isInactiveMotionDetail(clause)) {
        return compactDecisionText(clause);
      }
      continue;
    }

    const std::string module = trimCopy(clause.substr(0, separator));
    const std::string detail = trimCopy(clause.substr(separator + 1));
    if (isInactiveMotionDetail(detail)) {
      continue;
    }
    return compactDecisionText(compactModuleName(module) + ": " + detail);
  }
  return "BVP: no active constraint";
}

std::string extractStatus(const std::string & value, const std::string & prefix)
{
  const std::size_t begin = value.find(prefix);
  if (begin == std::string::npos) {
    return "";
  }
  std::size_t end = value.find('"', begin);
  if (end == std::string::npos) {
    end = value.find(';', begin);
  }
  if (end == std::string::npos) {
    end = value.size();
  }
  return value.substr(begin, end - begin);
}

std::string detailAfterPrefix(const std::string & status, const std::string & prefix)
{
  if (status.compare(0, prefix.size(), prefix) != 0) {
    return status;
  }
  std::string detail = status.substr(prefix.size());
  while (!detail.empty() && (detail.front() == ' ' || detail.front() == ':')) {
    detail.erase(detail.begin());
  }
  return detail;
}

bool isInactiveStatus(const std::string & status)
{
  return status.empty() || contains(status, "disabled") || contains(status, "pass-through") ||
         contains(status, "no request") || contains(status, "no static target") ||
         contains(status, "no dynamic target") || contains(status, "inactive");
}

std::string join(const std::vector<std::string> & values, const std::string & separator)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) {
      stream << separator;
    }
    stream << values[index];
  }
  return stream.str();
}

QString routeStateName(const std::uint8_t state)
{
  using RouteState = autoware_internal_planning_msgs::RouteState;
  switch (state) {
    case RouteState::INITIALIZING:
      return "INITIALIZING";
    case RouteState::UNSET:
      return "UNSET";
    case RouteState::ROUTING:
      return "ROUTING";
    case RouteState::SET:
      return "SET";
    case RouteState::REROUTING:
      return "REROUTING";
    case RouteState::ARRIVED:
      return "ARRIVED";
    case RouteState::ABORTED:
      return "ABORTED";
    case RouteState::INTERRUPTED:
      return "INTERRUPTED";
    default:
      return "UNKNOWN";
  }
}

QString routeStateLevel(const std::uint8_t state)
{
  using RouteState = autoware_internal_planning_msgs::RouteState;
  if (state == RouteState::SET || state == RouteState::ROUTING ||
      state == RouteState::ARRIVED) {
    return "good";
  }
  if (state == RouteState::INITIALIZING || state == RouteState::REROUTING) {
    return "warning";
  }
  if (state == RouteState::ABORTED || state == RouteState::INTERRUPTED) {
    return "error";
  }
  return "neutral";
}

std::pair<std::string, std::string> planningFactorText(
  const autoware_internal_planning_msgs::PlanningFactor & factor)
{
  using PlanningFactor = autoware_internal_planning_msgs::PlanningFactor;
  std::string behavior;
  switch (factor.behavior) {
    case PlanningFactor::SLOW_DOWN:
      behavior = "SLOW DOWN";
      break;
    case PlanningFactor::STOP:
      behavior = "STOP";
      break;
    case PlanningFactor::SHIFT_LEFT:
      behavior = "SHIFT LEFT";
      break;
    case PlanningFactor::SHIFT_RIGHT:
      behavior = "SHIFT RIGHT";
      break;
    case PlanningFactor::TURN_LEFT:
      behavior = "TURN LEFT";
      break;
    case PlanningFactor::TURN_RIGHT:
      behavior = "TURN RIGHT";
      break;
    default:
      behavior = "";
      break;
  }
  std::string detail = compactModuleName(factor.module);
  const std::string compact_detail =
    compactDecisionText(factor.detail, kMaxPlanningFactorDetailLength);
  if (!compact_detail.empty()) {
    if (!detail.empty()) {
      detail += ": ";
    }
    detail += compact_detail;
  }
  return std::make_pair(behavior, compactDecisionText(detail));
}

int planningFactorPriority(const std::uint16_t behavior)
{
  using PlanningFactor = autoware_internal_planning_msgs::PlanningFactor;
  if (behavior == PlanningFactor::STOP) {
    return 4;
  }
  if (behavior == PlanningFactor::SLOW_DOWN) {
    return 3;
  }
  if (behavior == PlanningFactor::SHIFT_LEFT || behavior == PlanningFactor::SHIFT_RIGHT) {
    return 2;
  }
  if (behavior == PlanningFactor::TURN_LEFT || behavior == PlanningFactor::TURN_RIGHT) {
    return 1;
  }
  return 0;
}
}  // namespace

void PlanningStatusPanel::RateTracker::record(const ros::WallTime & now)
{
  samples.push_back(now);
  while (samples.size() > 2 && (now - samples.front()).toSec() > kRateWindowSec) {
    samples.pop_front();
  }
}

double PlanningStatusPanel::RateTracker::rateHz() const
{
  if (samples.size() < 2) {
    return 0.0;
  }
  const double duration = (samples.back() - samples.front()).toSec();
  return duration > 0.0 ? static_cast<double>(samples.size() - 1) / duration : 0.0;
}

bool PlanningStatusPanel::RateTracker::isFresh(
  const ros::WallTime & now, const double timeout_sec) const
{
  return !samples.empty() && (now - samples.back()).toSec() <= timeout_sec;
}

PlanningStatusPanel::PlanningStatusPanel(QWidget * parent) : rviz::Panel(parent)
{
  setObjectName("planningStatusPanel");
  setMinimumWidth(320);
  setStyleSheet(
    "#planningStatusPanel { background: #eceeef; }"
    "QGroupBox { color: #34383c; font: 700 13px 'DejaVu Sans';"
    " border: 1px solid #c7c9cc; margin-top: 7px; background: #f4f5f6; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }");

  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(8);

  auto * title = new QLabel("PLANNING MONITOR");
  title->setAlignment(Qt::AlignCenter);
  title->setStyleSheet(
    "QLabel { color: #2f3337; font: 800 17px 'DejaVu Sans'; padding: 7px;"
    " background: #d8dadd; border: 1px solid #b8bbc0; }");
  layout->addWidget(title);

  auto * rate_row = new QHBoxLayout();
  rate_row->setSpacing(8);
  rate_row->addWidget(
    makeStatusGroup("Input", "KINEMATIC", &kinematic_rate_label_));
  rate_row->addWidget(
    makeStatusGroup("Output", "MVP RAW TRAJECTORY", &trajectory_rate_label_));
  layout->addLayout(rate_row);
  auto * speed_row = new QHBoxLayout();
  speed_row->setSpacing(8);
  speed_row->addWidget(
    makeStatusGroup("Vehicle", "CURRENT SPEED", &vehicle_speed_label_));
  speed_row->addWidget(
    makeStatusGroup("Target", "TRAJECTORY SPEED", &trajectory_speed_label_));
  layout->addLayout(speed_row);
  layout->addWidget(
    makeStatusGroup("State", "CURRENT BEHAVIOR", &behavior_label_));
  layout->addWidget(
    makeStatusGroup("Reason", "DECISION SOURCE", &decision_label_));
  layout->addWidget(makeStatusGroup("Route", "ROUTING", &route_label_));
  layout->addStretch(1);

  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &PlanningStatusPanel::refreshPanel);
  refresh_timer_->start(200);

  kinematic_subscriber_ = node_handle_.subscribe(
    "/localization/kinematic_state", 20,
    &PlanningStatusPanel::kinematicCallback, this);
  mvp_raw_trajectory_subscriber_ = node_handle_.subscribe(
    "/planning/scenario_planning/lane_driving/motion_planning/"
    "motion_velocity_planner/raw_trajectory",
    20, &PlanningStatusPanel::mvpRawTrajectoryCallback, this);
  trajectory_subscriber_ = node_handle_.subscribe(
    "/control/trajectory",
    20, &PlanningStatusPanel::trajectoryCallback, this);
  module_status_subscriber_ = node_handle_.subscribe(
    "/planning/debug/bpp/module_status", 10,
    &PlanningStatusPanel::moduleStatusCallback, this);
  motion_status_subscriber_ = node_handle_.subscribe(
    "/planning/debug/behavior_velocity_planner/status", 10,
    &PlanningStatusPanel::motionStatusCallback, this);
  planning_factors_subscriber_ = node_handle_.subscribe(
    "/planning/planning_factors/behavior_velocity_planner", 10,
    &PlanningStatusPanel::planningFactorsCallback, this);
  stop_reasons_subscriber_ = node_handle_.subscribe(
    "/planning/scenario_planning/status/stop_reasons", 10,
    &PlanningStatusPanel::stopReasonsCallback, this);
  route_state_subscriber_ = node_handle_.subscribe(
    "/planning/mission_planning/state", 10,
    &PlanningStatusPanel::routeStateCallback, this);
  route_subscriber_ = node_handle_.subscribe(
    "/planning/mission_planning/route", 10,
    &PlanningStatusPanel::routeCallback, this);

  refreshPanel();
}

void PlanningStatusPanel::onInitialize()
{
  auto * dock = qobject_cast<rviz::PanelDockWidget *>(parentWidget());
  auto * frame = dynamic_cast<rviz::VisualizationFrame *>(vis_manager_->getWindowManager());
  if (dock == nullptr || frame == nullptr) {
    return;
  }
  frame->addDockWidget(Qt::RightDockWidgetArea, dock);
  dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  dock->setMinimumWidth(320);
}

void PlanningStatusPanel::kinematicCallback(
  const nav_msgs::Odometry::ConstPtr & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  kinematic_rate_.record(ros::WallTime::now());
  const auto & velocity = message->twist.twist.linear;
  const double speed_mps = std::sqrt(
    velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);
  vehicle_speed_kph_ = speed_mps * 3.6;
  vehicle_position_x_ = message->pose.pose.position.x;
  vehicle_position_y_ = message->pose.pose.position.y;
  kinematic_state_received_ = true;
  updateTrajectorySpeed();
}

void PlanningStatusPanel::mvpRawTrajectoryCallback(
  const autoware_planning_msgs::Trajectory::ConstPtr &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  mvp_raw_trajectory_rate_.record(ros::WallTime::now());
}

void PlanningStatusPanel::trajectoryCallback(
  const autoware_planning_msgs::Trajectory::ConstPtr & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  control_trajectory_rate_.record(ros::WallTime::now());
  latest_trajectory_ = *message;
  updateTrajectorySpeed();
}

void PlanningStatusPanel::moduleStatusCallback(const std_msgs::String::ConstPtr & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  updateDecisionFromModuleStatus(message->data);
  bpp_status_time_ = ros::WallTime::now();
}

void PlanningStatusPanel::motionStatusCallback(const std_msgs::String::ConstPtr & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  motion_status_ = summarizeMotionStatus(message->data);
  motion_status_time_ = ros::WallTime::now();
}

void PlanningStatusPanel::planningFactorsCallback(
  const autoware_internal_planning_msgs::PlanningFactorArray::ConstPtr & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  updateDecisionFromPlanningFactors(*message);
  factors_received_ = true;
  factor_time_ = ros::WallTime::now();
}

void PlanningStatusPanel::stopReasonsCallback(
  const tier4_planning_msgs::StopReasonArray::ConstPtr & message)
{
  std::vector<std::string> reasons;
  reasons.reserve(std::min(message->stop_reasons.size(), kMaxStopReasonCount + 1));
  std::size_t omitted_count = 0;
  for (const auto & stop_reason : message->stop_reasons) {
    if (!stop_reason.reason.empty()) {
      if (reasons.size() < kMaxStopReasonCount) {
        reasons.push_back(compactDecisionText(stop_reason.reason, 24));
      } else {
        ++omitted_count;
      }
    }
  }
  if (omitted_count > 0) {
    reasons.push_back("+" + std::to_string(omitted_count) + " more");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  stop_decision_ = join(reasons, ", ");
  stop_reasons_received_ = true;
  stop_reason_time_ = ros::WallTime::now();
}

void PlanningStatusPanel::routeStateCallback(
  const autoware_internal_planning_msgs::RouteState::ConstPtr & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  route_state_ = message->state;
  route_state_received_ = true;
}

void PlanningStatusPanel::routeCallback(
  const autoware_planning_msgs::LaneletRoute::ConstPtr & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  route_segment_count_ = message->segments.size();
  route_received_ = true;
}

void PlanningStatusPanel::updateDecisionFromModuleStatus(const std::string & status)
{
  const std::string lane_follow =
    extractStatus(status, "lane_following:");
  bpp_behavior_ = "LANE FOLLOW";
  bpp_decision_ = lane_follow.empty() ?
    "Following planned route" : detailAfterPrefix(lane_follow, "lane_following:");

  const std::string lane_change = extractStatus(status, "lane_change:");
  if (!isInactiveStatus(lane_change) &&
      (contains(lane_change, "route") || contains(lane_change, "applied") ||
       contains(lane_change, "activated=true") || contains(lane_change, "blocked"))) {
    bpp_behavior_ = contains(lane_change, "blocked") ? "LANE CHANGE WAIT" : "LANE CHANGE";
    bpp_decision_ = detailAfterPrefix(lane_change, "lane_change:");
  }

  const std::string static_avoidance =
    extractStatus(status, "static_obstacle_avoidance:");
  if (!isInactiveStatus(static_avoidance)) {
    bpp_behavior_ = contains(static_avoidance, "safe=False") ? "AVOIDANCE WAIT" : "AVOIDANCE";
    bpp_decision_ =
      detailAfterPrefix(static_avoidance, "static_obstacle_avoidance:");
  }

  const std::string dynamic_avoidance =
    extractStatus(status, "dynamic_obstacle_avoidance:");
  if (!isInactiveStatus(dynamic_avoidance)) {
    bpp_behavior_ = "AVOIDANCE";
    bpp_decision_ =
      detailAfterPrefix(dynamic_avoidance, "dynamic_obstacle_avoidance:");
  }

  const std::string start_planner = extractStatus(status, "start_planner:");
  if (!isInactiveStatus(start_planner) && contains(start_planner, "shift pull-out")) {
    bpp_behavior_ = "START PLANNER";
    bpp_decision_ = detailAfterPrefix(start_planner, "start_planner:");
  }

  const std::string goal_planner = extractStatus(status, "goal_planner:");
  if (!isInactiveStatus(goal_planner)) {
    bpp_behavior_ = "GOAL PLANNER";
    bpp_decision_ = detailAfterPrefix(goal_planner, "goal_planner:");
  }

  const std::string side_shift = extractStatus(status, "side_shift:");
  if (!isInactiveStatus(side_shift) && contains(side_shift, "requested=")) {
    bpp_behavior_ = "SIDE SHIFT";
    bpp_decision_ = detailAfterPrefix(side_shift, "side_shift:");
  }
}

void PlanningStatusPanel::updateDecisionFromPlanningFactors(
  const autoware_internal_planning_msgs::PlanningFactorArray & message)
{
  factor_behavior_.clear();
  factor_decision_.clear();
  int selected_priority = 0;
  for (const auto & factor : message.factors) {
    const int priority = planningFactorPriority(factor.behavior);
    if (priority < selected_priority || priority == 0) {
      continue;
    }
    const auto text = planningFactorText(factor);
    selected_priority = priority;
    factor_behavior_ = text.first;
    factor_decision_ = text.second;
  }
}

void PlanningStatusPanel::updateTrajectorySpeed()
{
  trajectory_speed_received_ = false;
  if (!kinematic_state_received_ || latest_trajectory_.points.empty()) {
    return;
  }

  double nearest_distance_squared = std::numeric_limits<double>::infinity();
  double nearest_velocity_mps = 0.0;
  for (const auto & point : latest_trajectory_.points) {
    const double dx = point.pose.position.x - vehicle_position_x_;
    const double dy = point.pose.position.y - vehicle_position_y_;
    const double distance_squared = dx * dx + dy * dy;
    if (!std::isfinite(distance_squared) ||
        !std::isfinite(point.longitudinal_velocity_mps) ||
        distance_squared >= nearest_distance_squared) {
      continue;
    }
    nearest_distance_squared = distance_squared;
    nearest_velocity_mps = point.longitudinal_velocity_mps;
  }

  if (!std::isfinite(nearest_distance_squared)) {
    return;
  }
  trajectory_speed_kph_ = std::abs(nearest_velocity_mps) * 3.6;
  trajectory_speed_received_ = true;
}

void PlanningStatusPanel::setStatusLabel(
  QLabel * label, const QString & text, const QString & level)
{
  QString background = "#e0e2e4";
  QString border = "#c2c5c8";
  QString foreground = "#4b4f54";
  if (level == "good") {
    background = "#73ef4f";
    border = "#58cf38";
    foreground = "#245319";
  } else if (level == "warning") {
    background = "#ffd166";
    border = "#e5ad32";
    foreground = "#5e470d";
  } else if (level == "stop") {
    background = "#ff9f43";
    border = "#dc7920";
    foreground = "#5c2c05";
  } else if (level == "error") {
    background = "#ff6b6b";
    border = "#d94b4b";
    foreground = "#5b1717";
  }
  label->setText(text);
  label->setStyleSheet(
    "QLabel { color: " + foreground + "; background: " + background +
    "; border: 1px solid " + border +
    "; padding: 7px; font: 800 14px 'DejaVu Sans'; }");
}

void PlanningStatusPanel::refreshPanel()
{
  const ros::WallTime now = ros::WallTime::now();
  std::lock_guard<std::mutex> lock(mutex_);

  const auto update_rate = [this, &now](QLabel * label, const RateTracker & tracker) {
    if (!tracker.isFresh(now, kSignalTimeoutSec)) {
      setStatusLabel(label, "NO SIGNAL", "error");
      return;
    }
    const double rate = tracker.rateHz();
    if (rate <= 0.0) {
      setStatusLabel(label, "MEASURING", "warning");
      return;
    }
    setStatusLabel(label, QString("%1 Hz").arg(rate, 0, 'f', 1), "good");
  };
  update_rate(kinematic_rate_label_, kinematic_rate_);
  update_rate(trajectory_rate_label_, mvp_raw_trajectory_rate_);
  if (kinematic_rate_.isFresh(now, kSignalTimeoutSec)) {
    setStatusLabel(
      vehicle_speed_label_,
      QString("%1 km/h").arg(vehicle_speed_kph_, 0, 'f', 1),
      "good");
  } else {
    setStatusLabel(vehicle_speed_label_, "NO SIGNAL", "error");
  }
  if (kinematic_rate_.isFresh(now, kSignalTimeoutSec) &&
      control_trajectory_rate_.isFresh(now, kSignalTimeoutSec) &&
      trajectory_speed_received_) {
    setStatusLabel(
      trajectory_speed_label_,
      QString("%1 km/h").arg(trajectory_speed_kph_, 0, 'f', 1),
      "good");
  } else {
    setStatusLabel(trajectory_speed_label_, "NO SIGNAL", "error");
  }

  std::string behavior = bpp_behavior_;
  std::string decision = bpp_decision_;
  QString behavior_level = "good";
  if (bpp_status_time_.isZero() ||
      (now - bpp_status_time_).toSec() > kDecisionTimeoutSec) {
    behavior = "WAITING";
    decision = "Waiting for planner status";
    behavior_level = "warning";
  }

  if (factors_received_ && (now - factor_time_).toSec() <= kDecisionTimeoutSec &&
      !factor_behavior_.empty()) {
    behavior = factor_behavior_;
    decision = factor_decision_;
    behavior_level = behavior == "STOP" ? "stop" : "warning";
  }
  if (stop_reasons_received_ &&
      (now - stop_reason_time_).toSec() <= kDecisionTimeoutSec &&
      !stop_decision_.empty()) {
    behavior = "STOP";
    decision = stop_decision_;
    behavior_level = "stop";
  } else if (!motion_status_.empty() && motion_status_ != "no active modules" &&
             (now - motion_status_time_).toSec() <= kDecisionTimeoutSec &&
             factor_behavior_.empty()) {
    decision = motion_status_;
  }

  decision = compactDecisionText(decision);

  setStatusLabel(
    behavior_label_, QString::fromStdString(behavior), behavior_level);
  setStatusLabel(
    decision_label_, QString::fromStdString(decision),
    behavior_level == "stop" ? "stop" : behavior_level);

  if (!route_state_received_) {
    setStatusLabel(route_label_, "NO SIGNAL", "error");
    return;
  }
  QString route_text = routeStateName(route_state_);
  using RouteState = autoware_internal_planning_msgs::RouteState;
  if (route_received_ && route_segment_count_ > 0 &&
      route_state_ != RouteState::ARRIVED) {
    route_text += QString("  |  %1 segments").arg(route_segment_count_);
  }
  setStatusLabel(route_label_, route_text, routeStateLevel(route_state_));
}

}  // namespace trajectory_validator

PLUGINLIB_EXPORT_CLASS(trajectory_validator::PlanningStatusPanel, rviz::Panel)
