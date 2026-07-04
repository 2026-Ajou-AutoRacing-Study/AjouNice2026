#include <cmath>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class EgoTfBroadcasterDegCorrection
{
public:
  EgoTfBroadcasterDegCorrection()
  {
    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("ego_topic", ego_topic_, "/Ego_topic");
    private_nh.param<std::string>("parent_frame", parent_frame_, "morai_map");
    private_nh.param<std::string>("child_frame", child_frame_, "base_link");
    private_nh.param<bool>("heading_in_degrees", heading_in_degrees_, true);
    private_nh.param<double>("heading_yaw_offset_deg", heading_yaw_offset_deg_, -1.37);

    heading_yaw_offset_rad_ = heading_yaw_offset_deg_ * M_PI / 180.0;
    ego_subscriber_ = node_handle_.subscribe(ego_topic_, 50, &EgoTfBroadcasterDegCorrection::egoCallback, this);

    ROS_INFO_STREAM("ego_tf_broadcaster_deg_correction subscribe: " << ego_topic_);
    ROS_INFO_STREAM("ego_tf_broadcaster_deg_correction publish TF: " << parent_frame_ << " -> " << child_frame_);
    ROS_INFO_STREAM("ego_tf_broadcaster_deg_correction heading correction: " << heading_yaw_offset_deg_
                                                                             << " deg; raw /Ego_topic is unchanged");
  }

private:
  void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr &msg)
  {
    if (!std::isfinite(msg->position.x) || !std::isfinite(msg->position.y) || !std::isfinite(msg->position.z) ||
        !std::isfinite(msg->heading))
    {
      ROS_WARN_THROTTLE(2.0, "Skipping /Ego_topic with non-finite position or heading");
      return;
    }

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    if (stamp == last_stamp_)
    {
      return;
    }
    last_stamp_ = stamp;

    const double raw_yaw = heading_in_degrees_ ? msg->heading * M_PI / 180.0 : msg->heading;
    const double corrected_yaw = raw_yaw + heading_yaw_offset_rad_;
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, corrected_yaw);
    rotation.normalize();

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = child_frame_;
    transform.transform.translation.x = msg->position.x;
    transform.transform.translation.y = msg->position.y;
    transform.transform.translation.z = msg->position.z;
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();

    tf_broadcaster_.sendTransform(transform);
  }

  ros::NodeHandle node_handle_;
  ros::Subscriber ego_subscriber_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string ego_topic_;
  std::string parent_frame_;
  std::string child_frame_;
  bool heading_in_degrees_ = true;
  double heading_yaw_offset_deg_ = -1.37;
  double heading_yaw_offset_rad_ = 0.0;
  ros::Time last_stamp_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_tf_broadcaster_deg_correction");
  EgoTfBroadcasterDegCorrection broadcaster;
  ros::spin();
  return 0;
}
