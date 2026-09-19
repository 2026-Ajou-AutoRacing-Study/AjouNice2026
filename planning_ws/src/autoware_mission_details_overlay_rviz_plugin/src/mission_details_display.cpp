#include "mission_details_display.hpp"
#include <pluginlib/class_list_macros.h>
#include <rviz/ogre_helpers/render_system.h>
#include <QPainter>
#include <sstream>

namespace autoware::mission_details_overlay_rviz_plugin {
MissionDetailsDisplay::MissionDetailsDisplay() {
  width_=new rviz::IntProperty("Width",170,"Overlay width",this,SLOT(update_size())); height_=new rviz::IntProperty("Height",100,"Overlay height",this,SLOT(update_size())); right_=new rviz::IntProperty("Right",10,"Right margin",this,SLOT(update_size())); top_=new rviz::IntProperty("Top",10,"Top margin",this,SLOT(update_size()));
  alpha_=new rviz::FloatProperty("Background Alpha",0.3,"Background alpha",this,SLOT(update_size())); bg_=new rviz::ColorProperty("Background Color",QColor(0,0,0),"Background",this,SLOT(update_size())); text_=new rviz::ColorProperty("Text Color",QColor(194,194,194),"Text",this,SLOT(update_size()));
  topic_=new rviz::RosTopicProperty("Remaining Distance and Time Topic","/planning/mission_remaining_distance_time","autoware_internal_msgs/MissionRemainingDistanceTime","Mission detail topic",this,SLOT(topic_updated_remaining_distance_time())); data_=std::make_unique<RemainingDistanceTimeDisplay>();
}
MissionDetailsDisplay::~MissionDetailsDisplay(){ sub_.shutdown(); overlay_.reset(); }
void MissionDetailsDisplay::onInitialize(){ rviz::Display::onInitialize(); rviz::RenderSystem::get()->prepareOverlays(scene_manager_); static int n=0; overlay_=std::make_shared<rviz_2d_overlay_plugins::OverlayObject>("MissionDetailsDisplay"+std::to_string(n++)); update_size(); if(isEnabled()) subscribe(); }
void MissionDetailsDisplay::onEnable(){ if(overlay_) overlay_->show(); subscribe(); }
void MissionDetailsDisplay::onDisable(){ sub_.shutdown(); if(overlay_) overlay_->hide(); }
void MissionDetailsDisplay::subscribe(){ sub_.shutdown(); if(isEnabled()) sub_=update_nh_.subscribe(topic_->getTopicStd(),10,&MissionDetailsDisplay::callback,this); }
void MissionDetailsDisplay::topic_updated_remaining_distance_time(){ subscribe(); }
void MissionDetailsDisplay::callback(const autoware_internal_msgs::MissionRemainingDistanceTime::ConstPtr & msg){ std::lock_guard<std::mutex> lock(mutex_); data_->update(msg); dirty_=true; queueRender(); }
void MissionDetailsDisplay::update_size(){ if(!overlay_) return; overlay_->updateTextureSize(width_->getInt(),height_->getInt()); overlay_->setDimensions(overlay_->getTextureWidth(),overlay_->getTextureHeight()); overlay_->setPosition(right_->getInt(),top_->getInt(),rviz_2d_overlay_plugins::HorizontalAlignment::RIGHT,rviz_2d_overlay_plugins::VerticalAlignment::TOP); dirty_=true; queueRender(); }
void MissionDetailsDisplay::draw(QImage & image){ QPainter p(&image); QColor c=bg_->getColor(); c.setAlphaF(alpha_->getFloat()); p.setBrush(c); p.setPen(Qt::NoPen); QRectF r(0,0,width_->getInt(),height_->getInt()); p.drawRoundedRect(r,r.height()/2,r.height()/2); data_->draw(p,r,text_->getColor()); }
void MissionDetailsDisplay::update(float,float){ std::lock_guard<std::mutex> lock(mutex_); if(!dirty_||!overlay_||!isEnabled()) return; auto buffer=overlay_->getBuffer(); QImage image=buffer.getQImage(*overlay_); image.fill(Qt::transparent); draw(image); dirty_=false; }
void MissionDetailsDisplay::reset(){ rviz::Display::reset(); if(overlay_) overlay_->hide(); }
}
PLUGINLIB_EXPORT_CLASS(autoware::mission_details_overlay_rviz_plugin::MissionDetailsDisplay, rviz::Display)
