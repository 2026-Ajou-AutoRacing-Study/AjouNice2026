#ifndef STRING_STAMPED_OVERLAY_DISPLAY_HPP_
#define STRING_STAMPED_OVERLAY_DISPLAY_HPP_
#ifndef Q_MOC_RUN
#include <autoware_internal_debug_msgs/StringStamped.h>
#include <rviz_2d_overlay_plugins/overlay_utils.hpp>
#include <rviz_2d_overlay_plugins/unfiltered_topic_display.hpp>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <mutex>
#endif
namespace autoware::string_stamped_rviz_plugin {
class StringStampedOverlayDisplay : public rviz_2d_overlay_plugins::UnfilteredTopicDisplay<autoware_internal_debug_msgs::StringStamped> { Q_OBJECT
public: StringStampedOverlayDisplay(); ~StringStampedOverlayDisplay() override;
void onInitialize() override; void onEnable() override; void onDisable() override;
private Q_SLOTS: void updateVisualization();
protected: void update(float,float) override; void processMessage(const autoware_internal_debug_msgs::StringStamped::ConstPtr) override;
private: rviz_2d_overlay_plugins::OverlayObject::SharedPtr overlay_; rviz::ColorProperty *color_; rviz::IntProperty *left_,*top_,*font_,*maximum_; rviz::FloatProperty *keep_,*erase_; std::mutex mutex_; std::string text_,last_; ros::Time last_stamp_; };
}
#endif
