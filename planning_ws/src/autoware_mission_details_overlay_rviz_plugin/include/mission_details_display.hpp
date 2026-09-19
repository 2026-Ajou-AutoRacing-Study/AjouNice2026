#ifndef MISSION_DETAILS_DISPLAY_HPP_
#define MISSION_DETAILS_DISPLAY_HPP_
#ifndef Q_MOC_RUN
#include "remaining_distance_time_display.hpp"
#include <autoware_internal_msgs/MissionRemainingDistanceTime.h>
#include <rviz/display.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz_2d_overlay_plugins/overlay_utils.hpp>
#include <memory>
#include <mutex>
#endif
namespace autoware::mission_details_overlay_rviz_plugin {
class MissionDetailsDisplay : public rviz::Display { Q_OBJECT
public: MissionDetailsDisplay(); ~MissionDetailsDisplay() override;
protected: void onInitialize() override; void onEnable() override; void onDisable() override; void update(float,float) override; void reset() override;
private Q_SLOTS: void update_size(); void topic_updated_remaining_distance_time();
private: void subscribe(); void callback(const autoware_internal_msgs::MissionRemainingDistanceTime::ConstPtr &); void draw(QImage &);
rviz_2d_overlay_plugins::OverlayObject::SharedPtr overlay_; ros::Subscriber sub_; std::unique_ptr<RemainingDistanceTimeDisplay> data_; std::mutex mutex_; bool dirty_{true};
rviz::IntProperty *width_,*height_,*right_,*top_; rviz::FloatProperty *alpha_; rviz::ColorProperty *bg_,*text_; rviz::RosTopicProperty *topic_;
}; }
#endif
