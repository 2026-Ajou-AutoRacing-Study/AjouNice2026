#ifndef AUTOWARE_UTILS_RCLCPP__POLLING_SUBSCRIBER_HPP_
#define AUTOWARE_UTILS_RCLCPP__POLLING_SUBSCRIBER_HPP_

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <boost/bind.hpp>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware_utils_rclcpp
{

inline std::uint32_t single_depth_sensor_qos() { return 1U; }

namespace polling_policy
{
template <typename MessageT> class Latest;
template <typename MessageT> class Newest;
template <typename MessageT> class All;
}

template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
class InterProcessPollingSubscriber : public PollingPolicy<MessageT>
{
public:
  using ConstPtr = typename MessageT::ConstPtr;
  using SharedPtr = std::shared_ptr<InterProcessPollingSubscriber<MessageT, PollingPolicy>>;

  explicit InterProcessPollingSubscriber(
    ros::NodeHandle * node, const std::string & topic_name, std::uint32_t queue_depth = 1U)
  {
    if (node == nullptr) { throw std::invalid_argument("polling subscriber needs NodeHandle"); }
    this->check_qos(queue_depth);
    ros::SubscribeOptions options = ros::SubscribeOptions::create<MessageT>(
      topic_name, queue_depth,
      boost::bind(&InterProcessPollingSubscriber::on_message, this, _1), ros::VoidPtr(),
      &callback_queue_);
    subscriber_ = node->subscribe(options);
  }

  static SharedPtr create_subscription(
    ros::NodeHandle * node, const std::string & topic_name, std::uint32_t queue_depth = 1U)
  {
    return std::make_shared<InterProcessPollingSubscriber<MessageT, PollingPolicy>>(
      node, topic_name, queue_depth);
  }

  ros::Subscriber subscriber() const { return subscriber_; }

private:
  template <typename> friend class polling_policy::Latest;
  template <typename> friend class polling_policy::Newest;
  template <typename> friend class polling_policy::All;

  void on_message(const ConstPtr & message)
  {
    messages_.push_back(message);
    receipt_times_.push_back(ros::Time::now());
  }

  bool take_one(ConstPtr & message, ros::Time & receipt_time)
  {
    callback_queue_.callOne(ros::WallDuration(0.0));
    if (messages_.empty()) { return false; }
    message = messages_.front(); messages_.pop_front();
    receipt_time = receipt_times_.front(); receipt_times_.pop_front();
    return true;
  }

  std::vector<ConstPtr> take_all(ros::Time & last_receipt)
  {
    while (callback_queue_.callOne(ros::WallDuration(0.0)) == ros::CallbackQueue::Called) {}
    std::vector<ConstPtr> result;
    while (!messages_.empty()) {
      result.push_back(messages_.front()); messages_.pop_front();
      last_receipt = receipt_times_.front(); receipt_times_.pop_front();
    }
    return result;
  }

  ros::CallbackQueue callback_queue_;
  ros::Subscriber subscriber_;
  std::deque<ConstPtr> messages_;
  std::deque<ros::Time> receipt_times_;
};

namespace polling_policy
{
template <typename MessageT>
class Latest
{
protected:
  void check_qos(std::uint32_t depth)
  {
    if (depth > 1U) { throw std::invalid_argument("Latest polling requires queue depth <= 1"); }
  }
public:
  typename MessageT::ConstPtr take_data()
  {
    auto * owner = static_cast<InterProcessPollingSubscriber<MessageT, Latest> *>(this);
    typename MessageT::ConstPtr message; ros::Time receipt;
    if (owner->take_one(message, receipt)) { data_ = message; timestamp_ = receipt; }
    return data_;
  }
  std::optional<ros::Time> last_taken_data_timestamp() const { return timestamp_; }
private:
  typename MessageT::ConstPtr data_;
  std::optional<ros::Time> timestamp_;
};

template <typename MessageT>
class Newest
{
protected:
  void check_qos(std::uint32_t depth)
  {
    if (depth > 1U) { throw std::invalid_argument("Newest polling requires queue depth <= 1"); }
  }
public:
  typename MessageT::ConstPtr take_data()
  {
    auto * owner = static_cast<InterProcessPollingSubscriber<MessageT, Newest> *>(this);
    typename MessageT::ConstPtr message; ros::Time receipt;
    if (!owner->take_one(message, receipt)) { timestamp_ = std::nullopt; return typename MessageT::ConstPtr(); }
    timestamp_ = receipt; return message;
  }
  std::optional<ros::Time> last_taken_data_timestamp() const { return timestamp_; }
private:
  std::optional<ros::Time> timestamp_;
};

template <typename MessageT>
class All
{
protected:
  void check_qos(std::uint32_t) {}
public:
  std::vector<typename MessageT::ConstPtr> take_data()
  {
    auto * owner = static_cast<InterProcessPollingSubscriber<MessageT, All> *>(this);
    ros::Time receipt; auto messages = owner->take_all(receipt);
    if (messages.empty()) { timestamp_ = std::nullopt; } else { timestamp_ = receipt; }
    return messages;
  }
  std::optional<ros::Time> last_taken_data_timestamp() const { return timestamp_; }
private:
  std::optional<ros::Time> timestamp_;
};
}  // namespace polling_policy
}  // namespace autoware_utils_rclcpp

#endif
