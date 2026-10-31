#include "bringup/scoped_image_display.hpp"

#include <atomic>
#include <pluginlib/class_list_macros.h>
#include <rviz/display_context.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/status_property.h>

namespace bringup
{
namespace
{
std::atomic<unsigned> display_id{0};
}

ScopedImageDisplay::ScopedImageDisplay()
: transport_parameters_(ros::NodeHandle("~"),
    "image_displays/display_" + std::to_string(display_id.fetch_add(1)))
{
}

void ScopedImageDisplay::subscribe()
{
  if (!isEnabled()) return;
  try {
    // Release the prior subscriber/server before rebuilding on topic changes.
    // Unique instances also work when several displays are all named "Image".
    tf_filter_.reset();
    sub_.reset();
    sub_.reset(new image_transport::SubscriberFilter());
    if (!topic_property_->getTopicStd().empty() && !transport_property_->getStdString().empty()) {
      ros::TransportHints hints;
      if (unreliable_property_->getBool()) hints.unreliable();
      sub_->subscribe(*it_, topic_property_->getTopicStd(), queue_size_property_->getInt(),
        image_transport::TransportHints(transport_property_->getStdString(), hints,
                                        transport_parameters_));
      if (targetFrame_.empty()) {
        sub_->registerCallback(boost::function<void(const sensor_msgs::Image::ConstPtr&)>(
          [this](const sensor_msgs::Image::ConstPtr& msg) { incomingMessage(msg); }));
      } else {
        tf_filter_.reset(new tf2_ros::MessageFilter<sensor_msgs::Image>(
          *sub_, *context_->getTF2BufferPtr(), targetFrame_, queue_size_property_->getInt(), update_nh_));
        tf_filter_->registerCallback(boost::function<void(const sensor_msgs::Image::ConstPtr&)>(
          [this](const sensor_msgs::Image::ConstPtr& msg) { incomingMessage(msg); }));
        tf_filter_->registerFailureCallback(
          boost::function<void(const sensor_msgs::Image::ConstPtr&, tf2_ros::FilterFailureReason)>(
            [this](const sensor_msgs::Image::ConstPtr& msg, tf2_ros::FilterFailureReason reason) {
              failedMessage(msg, reason);
            }));
      }
    }
    setStatus(rviz::StatusProperty::Ok, "Topic", "OK");
  } catch (const ros::Exception& error) {
    setStatus(rviz::StatusProperty::Error, "Topic", QString("Error subscribing: ") + error.what());
  } catch (const image_transport::Exception& error) {
    setStatus(rviz::StatusProperty::Error, "Topic", QString("Error subscribing: ") + error.what());
  }
}
}  // namespace bringup

PLUGINLIB_EXPORT_CLASS(bringup::ScopedImageDisplay, rviz::Display)
