#pragma once

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <ros/message_traits.h>
#include <ros/ros.h>

#include <cstdint>
#include <atomic>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <utility>

namespace autoware_utils_debug::ros1
{
// Header-only inline storage is process-wide (not factory-local) under C++17.
// It deliberately contains no ROS node/endpoint object.
inline std::atomic<std::uint64_t> process_wide_publisher_token_counter{1};

inline std::uint64_t next_process_wide_publisher_token()
{
  std::uint64_t value = process_wide_publisher_token_counter.load(std::memory_order_relaxed);
  while (value != 0) {
    const std::uint64_t next = value == std::numeric_limits<std::uint64_t>::max() ? 0 : value + 1;
    if (process_wide_publisher_token_counter.compare_exchange_weak(
        value, next, std::memory_order_relaxed, std::memory_order_relaxed)) {
      return value;
    }
  }
  throw std::overflow_error("PublisherToken process-wide space exhausted");
}

struct PublisherToken {
  std::uint64_t value{0};
  bool operator<(const PublisherToken & other) const { return value < other.value; }
};

template <typename MessageT> class IdentifiedPublisher
{
public:
  PublisherToken token() const { return token_; }
  const ros::Publisher & raw() const { return publisher_; }
  std::string getTopic() const { return publisher_.getTopic(); }
  std::uint32_t getNumSubscribers() const { return publisher_.getNumSubscribers(); }
  bool valid() const { return static_cast<bool>(publisher_); }
  void publish(const MessageT & message) const { publisher_.publish(message); }
private:
  IdentifiedPublisher(ros::Publisher publisher, PublisherToken token)
  : publisher_(std::move(publisher)), token_(token) {}
  ros::Publisher publisher_;
  PublisherToken token_;
  friend class PublisherFactory;
};

class PublisherFactory
{
public:
  template <typename MessageT>
  IdentifiedPublisher<MessageT> advertise(
    ros::NodeHandle & node, const std::string & topic, const std::uint32_t queue_size,
    const bool latch = false)
  {
    static_assert(ros::message_traits::IsMessage<MessageT>::value, "ROS1 message required");
    if (queue_size == 0) throw std::invalid_argument("invalid primary endpoint");
    ros::Publisher publisher = node.advertise<MessageT>(topic, queue_size, latch);
    if (!publisher) throw std::runtime_error("primary advertise returned invalid publisher");
    return IdentifiedPublisher<MessageT>(std::move(publisher), PublisherToken{next_process_wide_publisher_token()});
  }
};

class CallerNodeContext final
{
public:
  using Ptr = std::shared_ptr<CallerNodeContext>;
  static Ptr borrow(const std::shared_ptr<ros::NodeHandle> & owner)
  {
    if (!owner) throw std::invalid_argument("non-null caller-owned NodeHandle required");
    return Ptr(new CallerNodeContext(owner));
  }
  CallerNodeContext(const CallerNodeContext &) = delete;
  CallerNodeContext & operator=(const CallerNodeContext &) = delete;
  ros::NodeHandle & require_alive() const
  {
    const auto owner = node_.lock();
    if (!owner) throw std::logic_error("caller-owned NodeHandle expired");
    return *owner;
  }
  template <typename MessageT> ros::Publisher advertise_default(const std::string & topic)
  {
    require_alive();
    return topic.compare(0, 2, "~/") == 0 ?
      private_node_.advertise<MessageT>(topic.substr(2), 1, false) :
      require_alive().advertise<MessageT>(topic, 1, false);
  }
private:
  explicit CallerNodeContext(const std::shared_ptr<ros::NodeHandle> & owner)
  : node_(owner), private_node_("~") {}
  std::weak_ptr<ros::NodeHandle> node_;
  ros::NodeHandle private_node_;
};

template <typename MessageT> class DefaultPublisherCache
{
public:
  DefaultPublisherCache(CallerNodeContext::Ptr context, std::string ns)
  : context_(std::move(context)), ns_(std::move(ns))
  {
    if (!context_) throw std::invalid_argument("context required");
  }
  void publish(const std::string & name, const MessageT & message)
  {
    context_->require_alive();
    auto it = publishers_.find(name);
    if (it == publishers_.end()) {
      it = publishers_.emplace(name, context_->advertise_default<MessageT>(ns_ + "/" + name)).first;
    }
    context_->require_alive();
    it->second.publish(message);
  }
  std::size_t size() const { return publishers_.size(); }
private:
  CallerNodeContext::Ptr context_;
  std::string ns_;
  std::unordered_map<std::string, ros::Publisher> publishers_;
};

struct SourceDefaultQos {
  bool keep_last{true}; std::uint32_t depth{1}; bool reliable{true}; bool volatile_durability{true};
  bool default_deadline{true}; bool default_lifespan{true}; bool automatic_liveliness{true}; bool default_lease{true};
};
struct Ros1Qos { std::uint32_t queue_size; bool latch; };
inline Ros1Qos only_source_default_qos(const SourceDefaultQos & qos = {})
{
  if (!(qos.keep_last && qos.depth == 1 && qos.reliable && qos.volatile_durability &&
    qos.default_deadline && qos.default_lifespan && qos.automatic_liveliness && qos.default_lease)) {
    throw std::invalid_argument("unsupported ROS2 QoS profile");
  }
  return {1, false};
}

template <typename AuxiliaryT> class PublishedTimeCache
{
public:
  PublishedTimeCache(CallerNodeContext::Ptr context, std::string suffix)
  : context_(std::move(context)), suffix_(std::move(suffix)), qos_(only_source_default_qos())
  {
    if (!context_) throw std::invalid_argument("context required");
  }
  template <typename PrimaryT> ros::Publisher & observe(const IdentifiedPublisher<PrimaryT> & primary)
  {
    context_->require_alive();
    if (!primary.valid() || primary.token().value == 0 || primary.getTopic().empty()) {
      throw std::logic_error("invalid identified primary publisher");
    }
    auto found = entries_.find(primary.token());
    if (found != entries_.end()) {
      if (found->second.topic != primary.getTopic()) throw std::logic_error("publisher token reused");
      return found->second.publisher;
    }
    Entry entry{primary.getTopic(), context_->require_alive().advertise<AuxiliaryT>(
      primary.getTopic() + suffix_, qos_.queue_size, qos_.latch)};
    if (!entry.publisher) throw std::runtime_error("auxiliary advertise returned invalid publisher");
    return entries_.emplace(primary.token(), std::move(entry)).first->second.publisher;
  }
  std::size_t size() const { return entries_.size(); }
private:
  struct Entry { std::string topic; ros::Publisher publisher; };
  CallerNodeContext::Ptr context_;
  std::string suffix_;
  Ros1Qos qos_;
  std::map<PublisherToken, Entry> entries_;
};
}  // namespace autoware_utils_debug::ros1
