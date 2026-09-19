#pragma once

#include <rviz/default_plugin/marker_display.h>
#include <rviz/display_context.h>
#include <boost/make_shared.hpp>

// Own the ROS1 marker renderer without subscribing to a second ROS topic.
// Object displays feed the same markers produced by the original geometry code.
class Ros1MarkerCommon : public rviz::MarkerDisplay
{
public:
  explicit Ros1MarkerCommon(rviz::Display * owner) : owner_(owner) {}
  void initialize(rviz::DisplayContext * context, Ogre::SceneNode *)
  {
    rviz::Display::initialize(context);
    setFixedFrame(context->getFixedFrame());
    setEnabled(owner_->isEnabled());
  }
  void load(const rviz::Config & config) override { rviz::MarkerDisplay::load(config); }
  void clearMarkers() { reset(); }
  void deleteMarker(const rviz::MarkerID & id)
  {
    auto marker = boost::make_shared<visualization_msgs::Marker>();
    marker->header.frame_id = fixed_frame_.toStdString();
    marker->ns = id.first;
    marker->id = id.second;
    marker->action = visualization_msgs::Marker::DELETE;
    addMessage(marker);
  }
  void addMessage(const visualization_msgs::MarkerArray::ConstPtr & markers)
  {
    incomingMarkerArray(markers);
  }
  void addMessage(const visualization_msgs::Marker::ConstPtr & marker)
  {
    auto array = boost::make_shared<visualization_msgs::MarkerArray>();
    array->markers.push_back(*marker);
    incomingMarkerArray(array);
  }
protected:
  void subscribe() override {}
  void unsubscribe() override {}
private:
  rviz::Display * owner_;
};
