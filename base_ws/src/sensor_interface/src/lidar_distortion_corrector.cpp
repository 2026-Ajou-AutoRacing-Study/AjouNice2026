#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <geometry_msgs/Vector3Stamped.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sensor_interface/lidar_preprocess_utils.hpp"

class LidarDistortionCorrector
{
public:
  LidarDistortionCorrector();

private:
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  sensor_interface::MotionBuffer motion_buffer_;

  ros::Subscriber cloud_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber ego_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Publisher cloud_publisher_;

  std::string cloud_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  std::string base_frame_id_;
  std::string motion_mode_;
  std::string imu_topic_;
  std::string ego_topic_;
  std::string odom_topic_;
  std::string morai_velocity_frame_;
  std::string odom_twist_frame_;
  std::string correction_mode_;
  std::string field_x_;
  std::string field_y_;
  std::string field_z_;
  std::string field_time_;

  double motion_timeout_sec_;
  double imu_timeout_sec_;
  double wait_for_transform_sec_;
  double buffer_duration_sec_;
  int subscriber_queue_size_;
  bool warned_morai_auto_velocity_;

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr &msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

  geometry_msgs::TransformStamped lookupCloudTransform(const std_msgs::Header &header) const;
  bool getReferenceMotion(const ros::Time &stamp, double *vx, double *vy, double *wz);
  void velocityToBase(const std::string &frame_mode, const double yaw, const double input_x,
                      const double input_y, double *vx, double *vy) const;
};

LidarDistortionCorrector::LidarDistortionCorrector()
  : node_handle_()
  , private_node_handle_("~")
  , tf_listener_(tf_buffer_)
  , motion_buffer_(2.0)
  , motion_timeout_sec_(0.2)
  , imu_timeout_sec_(0.2)
  , wait_for_transform_sec_(0.2)
  , buffer_duration_sec_(2.0)
  , subscriber_queue_size_(10)
  , warned_morai_auto_velocity_(false)
{
  private_node_handle_.param("cloud_topic", cloud_topic_, std::string("/sensor/lidar1"));
  private_node_handle_.param("output_topic", output_topic_, std::string("/sensor/lidar1/deskewed"));
  private_node_handle_.param("output_frame_id", output_frame_id_, std::string("base_link"));
  private_node_handle_.param("base_frame_id", base_frame_id_, std::string("base_link"));
  private_node_handle_.param("motion_mode", motion_mode_, std::string("morai"));
  private_node_handle_.param("imu_topic", imu_topic_, std::string("/sensor/imu"));
  private_node_handle_.param("ego_topic", ego_topic_, std::string("/Ego_topic"));
  private_node_handle_.param("odom_topic", odom_topic_, std::string("/localization/odom"));
  private_node_handle_.param("morai_velocity_frame", morai_velocity_frame_, std::string("base_link"));
  private_node_handle_.param("odom_twist_frame", odom_twist_frame_, std::string("base_link"));
  private_node_handle_.param("correction_mode", correction_mode_, std::string("2d"));
  private_node_handle_.param("field_x", field_x_, std::string("x"));
  private_node_handle_.param("field_y", field_y_, std::string("y"));
  private_node_handle_.param("field_z", field_z_, std::string("z"));
  private_node_handle_.param("field_time", field_time_, std::string("time"));
  private_node_handle_.param("motion_timeout_sec", motion_timeout_sec_, 0.2);
  private_node_handle_.param("imu_timeout_sec", imu_timeout_sec_, 0.2);
  private_node_handle_.param("wait_for_transform_sec", wait_for_transform_sec_, 0.2);
  private_node_handle_.param("buffer_duration_sec", buffer_duration_sec_, 2.0);
  private_node_handle_.param("subscriber_queue_size", subscriber_queue_size_, 10);

  motion_buffer_.setDuration(buffer_duration_sec_);

  if (correction_mode_ != "2d")
  {
    ROS_WARN_STREAM("Only correction_mode=2d is implemented. Requested mode: " << correction_mode_);
  }

  imu_subscriber_ = node_handle_.subscribe(imu_topic_, static_cast<uint32_t>(subscriber_queue_size_),
                                           &LidarDistortionCorrector::imuCallback, this);

  if (motion_mode_ == "morai")
  {
    ego_subscriber_ = node_handle_.subscribe(ego_topic_, static_cast<uint32_t>(subscriber_queue_size_),
                                             &LidarDistortionCorrector::egoCallback, this);
  }
  else if (motion_mode_ == "odom")
  {
    odom_subscriber_ = node_handle_.subscribe(odom_topic_, static_cast<uint32_t>(subscriber_queue_size_),
                                              &LidarDistortionCorrector::odomCallback, this);
  }
  else
  {
    ROS_FATAL_STREAM("Unsupported motion_mode: " << motion_mode_ << " (expected morai or odom)");
    ros::shutdown();
    return;
  }

  cloud_subscriber_ = node_handle_.subscribe(cloud_topic_, 2, &LidarDistortionCorrector::cloudCallback, this);
  cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);

  ROS_INFO_STREAM("lidar_distortion_corrector " << cloud_topic_ << " -> " << output_topic_
                                                << " frame=" << output_frame_id_
                                                << " motion_mode=" << motion_mode_
                                                << " imu=" << imu_topic_);
}

void LidarDistortionCorrector::velocityToBase(const std::string &frame_mode, const double yaw,
                                              const double input_x, const double input_y,
                                              double *vx, double *vy) const
{
  if (frame_mode == "world" || frame_mode == "map" || frame_mode == "morai_map")
  {
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    *vx = cos_yaw * input_x + sin_yaw * input_y;
    *vy = -sin_yaw * input_x + cos_yaw * input_y;
    return;
  }

  *vx = input_x;
  *vy = input_y;
}

void LidarDistortionCorrector::egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr &msg)
{
  const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  const double yaw = sensor_interface::normalizeAngle(msg->heading * M_PI / 180.0);

  double vx = 0.0;
  double vy = 0.0;
  const std::string velocity_frame = morai_velocity_frame_ == "auto" ? "base_link" : morai_velocity_frame_;
  if (morai_velocity_frame_ == "auto" && !warned_morai_auto_velocity_)
  {
    ROS_WARN("morai_velocity_frame=auto is treated as base_link until a moving-frame check proves otherwise");
    warned_morai_auto_velocity_ = true;
  }
  velocityToBase(velocity_frame, yaw, msg->velocity.x, msg->velocity.y, &vx, &vy);

  sensor_interface::MotionSample sample;
  sample.stamp = stamp;
  sample.x = msg->position.x;
  sample.y = msg->position.y;
  sample.yaw = yaw;
  sample.vx = vx;
  sample.vy = vy;
  motion_buffer_.addMotion(sample);
}

void LidarDistortionCorrector::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  const double yaw = sensor_interface::normalizeAngle(tf2::getYaw(msg->pose.pose.orientation));

  double vx = 0.0;
  double vy = 0.0;
  velocityToBase(odom_twist_frame_, yaw, msg->twist.twist.linear.x, msg->twist.twist.linear.y, &vx, &vy);

  sensor_interface::MotionSample sample;
  sample.stamp = stamp;
  sample.x = msg->pose.pose.position.x;
  sample.y = msg->pose.pose.position.y;
  sample.yaw = yaw;
  sample.vx = vx;
  sample.vy = vy;
  motion_buffer_.addMotion(sample);
}

void LidarDistortionCorrector::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  double wz = msg->angular_velocity.z;

  if (!msg->header.frame_id.empty() && msg->header.frame_id != base_frame_id_)
  {
    try
    {
      const geometry_msgs::TransformStamped transform =
          tf_buffer_.lookupTransform(base_frame_id_, msg->header.frame_id, ros::Time(0),
                                     ros::Duration(wait_for_transform_sec_));
      geometry_msgs::Vector3Stamped input;
      geometry_msgs::Vector3Stamped output;
      input.header = msg->header;
      input.vector = msg->angular_velocity;
      tf2::doTransform(input, output, transform);
      wz = output.vector.z;
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_WARN_THROTTLE(1.0, "Failed to transform IMU angular velocity to %s: %s", base_frame_id_.c_str(),
                        ex.what());
      return;
    }
  }

  sensor_interface::ImuSample sample;
  sample.stamp = stamp;
  sample.wz = wz;
  motion_buffer_.addImu(sample);
}

geometry_msgs::TransformStamped LidarDistortionCorrector::lookupCloudTransform(const std_msgs::Header &header) const
{
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = header.stamp;
  transform.header.frame_id = output_frame_id_;
  transform.child_frame_id = header.frame_id;
  transform.transform.rotation.w = 1.0;

  if (header.frame_id.empty() || header.frame_id == output_frame_id_)
  {
    return transform;
  }

  return tf_buffer_.lookupTransform(output_frame_id_, header.frame_id, ros::Time(0),
                                    ros::Duration(wait_for_transform_sec_));
}

bool LidarDistortionCorrector::getReferenceMotion(const ros::Time &stamp, double *vx, double *vy, double *wz)
{
  sensor_interface::MotionSample motion;
  if (!motion_buffer_.getMotion(stamp, motion_timeout_sec_, &motion))
  {
    ROS_WARN_THROTTLE(1.0, "No ego motion sample near cloud stamp %.6f", stamp.toSec());
    return false;
  }

  double imu_wz = 0.0;
  if (!motion_buffer_.getImuWz(stamp, imu_timeout_sec_, &imu_wz))
  {
    ROS_WARN_THROTTLE(1.0, "No IMU angular velocity sample near cloud stamp %.6f", stamp.toSec());
    return false;
  }

  *vx = motion.vx;
  *vy = motion.vy;
  *wz = imu_wz;
  return true;
}

void LidarDistortionCorrector::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (msg->is_bigendian)
  {
    ROS_WARN_THROTTLE(1.0, "Skipping big-endian PointCloud2; only little-endian clouds are supported");
    return;
  }

  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  if (!getReferenceMotion(msg->header.stamp, &vx, &vy, &wz))
  {
    return;
  }

  geometry_msgs::TransformStamped cloud_transform;
  try
  {
    cloud_transform = lookupCloudTransform(msg->header);
  }
  catch (const tf2::TransformException &ex)
  {
    ROS_WARN_THROTTLE(1.0, "Failed to transform cloud from %s to %s: %s", msg->header.frame_id.c_str(),
                      output_frame_id_.c_str(), ex.what());
    return;
  }
  tf2::Transform cloud_tf;
  tf2::fromMsg(cloud_transform.transform, cloud_tf);

  sensor_msgs::PointCloud2 output = *msg;
  output.header.stamp = msg->header.stamp;
  output.header.frame_id = output_frame_id_;

  const sensor_msgs::PointField *x_field = sensor_interface::findField(output, field_x_);
  const sensor_msgs::PointField *y_field = sensor_interface::findField(output, field_y_);
  const sensor_msgs::PointField *z_field = sensor_interface::findField(output, field_z_);
  const sensor_msgs::PointField *time_field = sensor_interface::findField(output, field_time_);

  if (x_field == nullptr || y_field == nullptr || z_field == nullptr)
  {
    ROS_WARN_THROTTLE(1.0, "Skipping cloud without x/y/z fields");
    return;
  }
  if (x_field->datatype != sensor_msgs::PointField::FLOAT32 ||
      y_field->datatype != sensor_msgs::PointField::FLOAT32 ||
      z_field->datatype != sensor_msgs::PointField::FLOAT32)
  {
    ROS_WARN_THROTTLE(1.0, "Skipping cloud because x/y/z fields are not FLOAT32");
    return;
  }
  if (time_field == nullptr)
  {
    ROS_WARN_THROTTLE(1.0, "Cloud has no time field; only frame transform will be applied");
  }

  const size_t point_count = static_cast<size_t>(output.width) * static_cast<size_t>(output.height);
  const size_t required_size = point_count * static_cast<size_t>(output.point_step);
  if (output.data.size() < required_size)
  {
    ROS_WARN_THROTTLE(1.0, "Skipping malformed cloud: data size is smaller than width*height*point_step");
    return;
  }

  double min_dt = std::numeric_limits<double>::infinity();
  double max_dt = -std::numeric_limits<double>::infinity();
  size_t corrected_points = 0;

  for (size_t i = 0; i < point_count; ++i)
  {
    const size_t point_offset = i * static_cast<size_t>(output.point_step);
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!sensor_interface::readFieldAsDouble(output, *x_field, point_offset, &x) ||
        !sensor_interface::readFieldAsDouble(output, *y_field, point_offset, &y) ||
        !sensor_interface::readFieldAsDouble(output, *z_field, point_offset, &z))
    {
      continue;
    }
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    {
      continue;
    }

    double dt = 0.0;
    if (time_field != nullptr)
    {
      double point_time_offset = 0.0;
      if (sensor_interface::readFieldAsDouble(output, *time_field, point_offset, &point_time_offset))
      {
        dt = point_time_offset;
        min_dt = std::min(min_dt, dt);
        max_dt = std::max(max_dt, dt);
      }
    }

    const tf2::Vector3 transformed = cloud_tf * tf2::Vector3(x, y, z);
    sensor_interface::Point3 point{ transformed.x(), transformed.y(), transformed.z() };
    point = sensor_interface::compensatePoint2D(point, vx, vy, wz, dt);

    sensor_interface::writeFloat32Field(&output, *x_field, point_offset, point.x);
    sensor_interface::writeFloat32Field(&output, *y_field, point_offset, point.y);
    sensor_interface::writeFloat32Field(&output, *z_field, point_offset, point.z);
    ++corrected_points;
  }

  cloud_publisher_.publish(output);
  ROS_DEBUG_STREAM_THROTTLE(1.0, "deskewed " << corrected_points << " points from " << cloud_topic_
                                             << " time_span=[" << min_dt << ", " << max_dt
                                             << "] vx=" << vx << " vy=" << vy << " wz=" << wz);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_distortion_corrector");
  LidarDistortionCorrector node;
  ros::spin();
  return 0;
}
