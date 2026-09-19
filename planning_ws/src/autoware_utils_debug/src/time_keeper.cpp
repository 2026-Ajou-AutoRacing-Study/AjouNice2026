#include <autoware_utils_debug/time_keeper.hpp>

#include <ros/console.h>

#include <sstream>
#include <stdexcept>

namespace autoware_utils_debug
{
ProcessingTimeNode::ProcessingTimeNode(const std::string & name) : name_(name) {}
std::shared_ptr<ProcessingTimeNode> ProcessingTimeNode::add_child(const std::string & name)
{
  auto child = std::make_shared<ProcessingTimeNode>(name);
  child->parent_node_ = weak_from_this();
  child_nodes_.push_back(child);
  return child;
}
std::string ProcessingTimeNode::to_string() const
{
  std::ostringstream stream;
  std::function<void(const ProcessingTimeNode &, const std::string &, bool, bool)> render;
  render = [&](const ProcessingTimeNode & node, const std::string & prefix, bool last, bool root) {
    if (!root) stream << prefix << (last ? "└── " : "├── ");
    stream << node.name_ << " (" << node.processing_time_ << "ms)";
    if (!node.comment_.empty()) stream << " : " << node.comment_;
    stream << '\n';
    for (std::size_t i = 0; i < node.child_nodes_.size(); ++i) {
      render(*node.child_nodes_[i], prefix + (last ? "    " : "│   "), i + 1 == node.child_nodes_.size(), false);
    }
  };
  render(*this, "", true, true);
  return stream.str();
}
autoware_internal_debug_msgs::ProcessingTimeTree ProcessingTimeNode::to_msg() const
{
  autoware_internal_debug_msgs::ProcessingTimeTree tree;
  std::function<void(const ProcessingTimeNode &, int)> visit;
  visit = [&](const ProcessingTimeNode & node, int parent) {
    autoware_internal_debug_msgs::ProcessingTimeNode message;
    message.name = node.name_; message.processing_time = node.processing_time_;
    message.id = static_cast<int>(tree.nodes.size() + 1); message.parent_id = parent;
    message.comment = node.comment_; tree.nodes.emplace_back(message);
    for (const auto & child : node.child_nodes_) visit(*child, message.id);
  };
  visit(*this, 0);
  return tree;
}
std::weak_ptr<ProcessingTimeNode> ProcessingTimeNode::get_parent_node() const { return parent_node_; }
std::vector<std::shared_ptr<ProcessingTimeNode>> ProcessingTimeNode::get_child_nodes() const { return child_nodes_; }
void ProcessingTimeNode::set_time(double value) { processing_time_ = value; }
void ProcessingTimeNode::set_comment(const std::string & value) { comment_ = value; }
std::string ProcessingTimeNode::get_name() const { return name_; }
void TimeKeeper::add_reporter(std::ostream * os)
{
  if (!os) throw std::invalid_argument("ostream reporter is null");
  reporters_.emplace_back([os](const auto & node) { *os << "==========================\n" << node->to_string() << std::endl; });
}
void TimeKeeper::add_reporter(const ros::Publisher & publisher)
{
  if (!publisher) throw std::invalid_argument("publisher reporter is invalid");
  reporters_.emplace_back([publisher](const auto & node) { publisher.publish(node->to_msg()); });
}
void TimeKeeper::start_track(const std::string & name)
{
  if (!current_time_node_) { current_time_node_ = std::make_shared<ProcessingTimeNode>(name); root_node_ = current_time_node_; root_node_thread_id_ = std::this_thread::get_id(); }
  else if (root_node_thread_id_ != std::this_thread::get_id()) {
    const std::string warning =
      "TimeKeeper::start_track(" + name + ") is called from a different thread. Ignoring the call.";
    ROS_LOG(::ros::console::levels::Warn, "ros.TimeKeeper", "%s", warning.c_str());
    return;
  }
  else current_time_node_ = current_time_node_->add_child(name);
  stop_watch_.tic(name);
}
void TimeKeeper::comment(const std::string & value)
{
  if (!current_time_node_) throw std::runtime_error("start_track must precede comment");
  current_time_node_->set_comment(value);
}
void TimeKeeper::end_track(const std::string & name)
{
  if (root_node_thread_id_ != std::this_thread::get_id()) return;
  if (!current_time_node_ || current_time_node_->get_name() != name) throw std::runtime_error("end_track order mismatch");
  current_time_node_->set_time(stop_watch_.toc(name));
  current_time_node_ = current_time_node_->get_parent_node().lock();
  if (!current_time_node_) report();
}
void TimeKeeper::report()
{
  if (current_time_node_) throw std::runtime_error("report requires completed root");
  for (const auto & reporter : reporters_) reporter(root_node_);
  root_node_.reset(); current_time_node_.reset();
}
ScopedTimeTrack::ScopedTimeTrack(const std::string & name, TimeKeeper & keeper) : func_name_(name), time_keeper_(keeper) { time_keeper_.start_track(func_name_); }
ScopedTimeTrack::~ScopedTimeTrack() { time_keeper_.end_track(func_name_); }
}
