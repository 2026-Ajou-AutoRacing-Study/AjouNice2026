#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

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

class LidarAdvancedConcatFilter
{
public:
  LidarAdvancedConcatFilter();

private:
  typedef sensor_msgs::PointCloud2 PointCloudMsgT;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  sensor_interface::MotionBuffer motion_buffer_;

  ros::Subscriber cloud_subscriber1_;
  ros::Subscriber cloud_subscriber2_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber ego_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Publisher cloud_publisher_;
  PointCloudMsgT::ConstPtr latest_cloud1_;
  PointCloudMsgT::ConstPtr latest_cloud2_;
  ros::Time last_published_stamp1_;
  ros::Time last_published_stamp2_;

  std::vector<std::string> input_topics_;
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

  double motion_timeout_sec_;
  double imu_timeout_sec_;
  double wait_for_transform_sec_;
  double buffer_duration_sec_;
  double max_sync_interval_sec_;
  int subscriber_queue_size_;
  bool enable_motion_compensation_;
  bool warned_morai_auto_velocity_;

  void cloud1Callback(const PointCloudMsgT::ConstPtr &msg);
  void cloud2Callback(const PointCloudMsgT::ConstPtr &msg);
  void tryPublish();
  void publishPair(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2);
  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr &msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

  geometry_msgs::TransformStamped lookupCloudTransform(const std_msgs::Header &header) const;
  bool getReferenceMotion(const ros::Time &stamp, sensor_interface::MotionSample *motion);
  bool appendCompensatedCloud(const PointCloudMsgT &input, const ros::Time &reference_stamp,
                              const sensor_interface::MotionSample &reference_motion, PointCloudMsgT *output);
  void velocityToBase(const std::string &frame_mode, const double yaw, const double input_x,
                      const double input_y, double *vx, double *vy) const;
};

LidarAdvancedConcatFilter::LidarAdvancedConcatFilter()
  : node_handle_()
  , private_node_handle_("~")
  , tf_listener_(tf_buffer_)
  , motion_buffer_(2.0)
  , motion_timeout_sec_(0.2)
  , imu_timeout_sec_(0.2)
  , wait_for_transform_sec_(0.2)
  , buffer_duration_sec_(2.0)
  , max_sync_interval_sec_(0.01)
  , subscriber_queue_size_(2)
  , enable_motion_compensation_(true)
  , warned_morai_auto_velocity_(false)
{
  input_topics_.push_back("/sensor/lidar1/deskewed");
  input_topics_.push_back("/sensor/lidar2/deskewed");
  private_node_handle_.getParam("input_topics", input_topics_);

  private_node_handle_.param("output_topic", output_topic_, std::string("/sensor/lidar_concat/deskewed"));
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
  private_node_handle_.param("motion_timeout_sec", motion_timeout_sec_, 0.2);
  private_node_handle_.param("imu_timeout_sec", imu_timeout_sec_, 0.2);
  private_node_handle_.param("wait_for_transform_sec", wait_for_transform_sec_, 0.2);
  private_node_handle_.param("buffer_duration_sec", buffer_duration_sec_, 2.0);
  private_node_handle_.param("max_sync_interval_sec", max_sync_interval_sec_, 0.01);
  private_node_handle_.param("subscriber_queue_size", subscriber_queue_size_, 2);
  private_node_handle_.param("enable_motion_compensation", enable_motion_compensation_, true);

  if (input_topics_.size() != 2)
  {
    ROS_FATAL("lidar_advanced_concat_filter currently expects exactly two input topics");
    ros::shutdown();
    return;
  }

  motion_buffer_.setDuration(buffer_duration_sec_);
  std::transform(correction_mode_.begin(), correction_mode_.end(), correction_mode_.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (correction_mode_ != "2d" && correction_mode_ != "3d")
  {
    ROS_WARN_STREAM("Unsupported correction_mode=" << correction_mode_ << "; falling back to 2d");
    correction_mode_ = "2d";
  }

  imu_subscriber_ = node_handle_.subscribe(imu_topic_, static_cast<uint32_t>(subscriber_queue_size_),
                                           &LidarAdvancedConcatFilter::imuCallback, this);
  if (motion_mode_ == "morai")
  {
    ego_subscriber_ = node_handle_.subscribe(ego_topic_, static_cast<uint32_t>(subscriber_queue_size_),
                                             &LidarAdvancedConcatFilter::egoCallback, this);
  }
  else if (motion_mode_ == "odom")
  {
    odom_subscriber_ = node_handle_.subscribe(odom_topic_, static_cast<uint32_t>(subscriber_queue_size_),
                                              &LidarAdvancedConcatFilter::odomCallback, this);
  }
  else
  {
    ROS_FATAL_STREAM("Unsupported motion_mode: " << motion_mode_ << " (expected morai or odom)");
    ros::shutdown();
    return;
  }

  cloud_subscriber1_ = node_handle_.subscribe(input_topics_[0], static_cast<uint32_t>(subscriber_queue_size_),
                                              &LidarAdvancedConcatFilter::cloud1Callback, this);
  cloud_subscriber2_ = node_handle_.subscribe(input_topics_[1], static_cast<uint32_t>(subscriber_queue_size_),
                                              &LidarAdvancedConcatFilter::cloud2Callback, this);

  cloud_publisher_ = node_handle_.advertise<PointCloudMsgT>(output_topic_, 1);

  ROS_INFO_STREAM("lidar_advanced_concat_filter publishing " << output_topic_ << " from "
                                                             << input_topics_[0] << ", " << input_topics_[1]
                                                             << " motion_mode=" << motion_mode_
                                                             << " correction_mode=" << correction_mode_
                                                             << " max_sync_interval=" << max_sync_interval_sec_
                                                             << " motion_compensation="
                                                             << (enable_motion_compensation_ ? "true" : "false"));
}

void LidarAdvancedConcatFilter::velocityToBase(const std::string &frame_mode, const double yaw,
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

void LidarAdvancedConcatFilter::egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr &msg)
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
  sample.z = msg->position.z;
  sample.yaw = yaw;
  sample.vx = vx;
  sample.vy = vy;
  sample.vz = msg->velocity.z;
  sample.wx = msg->angular_velocity.x;
  sample.wy = msg->angular_velocity.y;
  sample.wz = msg->angular_velocity.z;
  motion_buffer_.addMotion(sample);
}

void LidarAdvancedConcatFilter::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
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
  sample.z = msg->pose.pose.position.z;
  sample.yaw = yaw;
  sample.vx = vx;
  sample.vy = vy;
  sample.vz = msg->twist.twist.linear.z;
  sample.wx = msg->twist.twist.angular.x;
  sample.wy = msg->twist.twist.angular.y;
  sample.wz = msg->twist.twist.angular.z;
  motion_buffer_.addMotion(sample);
}

void LidarAdvancedConcatFilter::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  geometry_msgs::Vector3 angular_velocity = msg->angular_velocity;

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
      angular_velocity = output.vector;
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
  sample.wx = angular_velocity.x;
  sample.wy = angular_velocity.y;
  sample.wz = angular_velocity.z;
  motion_buffer_.addImu(sample);
}

geometry_msgs::TransformStamped LidarAdvancedConcatFilter::lookupCloudTransform(const std_msgs::Header &header) const
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

bool LidarAdvancedConcatFilter::getReferenceMotion(const ros::Time &stamp,
                                                   sensor_interface::MotionSample *motion)
{
  if (motion == nullptr || !motion_buffer_.getMotion(stamp, motion_timeout_sec_, motion))
  {
    ROS_WARN_THROTTLE(10.0,
                      "No ego motion sample near concat reference stamp %.6f; using zero-motion fallback",
                      stamp.toSec());
    if (motion != nullptr)
    {
      *motion = sensor_interface::MotionSample();
      motion->stamp = stamp;
    }
    return motion != nullptr;
  }

  sensor_interface::ImuSample imu;
  if (motion_buffer_.getImu(stamp, imu_timeout_sec_, &imu))
  {
    motion->wx = imu.wx;
    motion->wy = imu.wy;
    motion->wz = imu.wz;
    return true;
  }

  ROS_WARN_THROTTLE(10.0,
                    "No IMU angular velocity sample near concat reference stamp %.6f; using angular velocity from %s",
                    stamp.toSec(), motion_mode_.c_str());
  return true;
}

bool LidarAdvancedConcatFilter::appendCompensatedCloud(const PointCloudMsgT &input,
                                                       const ros::Time &reference_stamp,
                                                       const sensor_interface::MotionSample &reference_motion,
                                                       PointCloudMsgT *output)
{
  if (output == nullptr)
  {
    return false;
  }
  if (input.is_bigendian)
  {
    ROS_WARN_THROTTLE(1.0, "Skipping big-endian PointCloud2; only little-endian clouds are supported");
    return false;
  }

  sensor_msgs::PointCloud2 cloud = input;
  const sensor_msgs::PointField *x_field = sensor_interface::findField(cloud, field_x_);
  const sensor_msgs::PointField *y_field = sensor_interface::findField(cloud, field_y_);
  const sensor_msgs::PointField *z_field = sensor_interface::findField(cloud, field_z_);
  if (x_field == nullptr || y_field == nullptr || z_field == nullptr)
  {
    ROS_WARN_THROTTLE(1.0, "Skipping cloud without x/y/z fields");
    return false;
  }
  if (x_field->datatype != sensor_msgs::PointField::FLOAT32 ||
      y_field->datatype != sensor_msgs::PointField::FLOAT32 ||
      z_field->datatype != sensor_msgs::PointField::FLOAT32)
  {
    ROS_WARN_THROTTLE(1.0, "Skipping cloud because x/y/z fields are not FLOAT32");
    return false;
  }

  geometry_msgs::TransformStamped cloud_transform;
  try
  {
    cloud_transform = lookupCloudTransform(input.header);
  }
  catch (const tf2::TransformException &ex)
  {
    ROS_WARN_THROTTLE(1.0, "Failed to transform cloud from %s to %s: %s", input.header.frame_id.c_str(),
                      output_frame_id_.c_str(), ex.what());
    return false;
  }
  tf2::Transform cloud_tf;
  tf2::fromMsg(cloud_transform.transform, cloud_tf);

  const size_t point_count = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
  const size_t required_size = point_count * static_cast<size_t>(cloud.point_step);
  if (cloud.data.size() < required_size)
  {
    ROS_WARN_THROTTLE(1.0, "Skipping malformed cloud: data size is smaller than width*height*point_step");
    return false;
  }

  const double dt = enable_motion_compensation_ ? (input.header.stamp - reference_stamp).toSec() : 0.0;
  const size_t output_start = output->data.size();
  output->data.resize(output_start + required_size);
  std::memcpy(output->data.data() + output_start, cloud.data.data(), required_size);

  const bool already_in_output_frame = input.header.frame_id == output_frame_id_ || input.header.frame_id.empty();
  if (already_in_output_frame && std::abs(dt) < 0.001)
  {
    return true;
  }

  Eigen::Matrix4d transform3d = Eigen::Matrix4d::Identity();
  double dx = 0.0;
  double dy = 0.0;
  double dyaw = 0.0;
  double cos_yaw = 1.0;
  double sin_yaw = 0.0;
  if (correction_mode_ == "3d")
  {
    const sensor_interface::Vector3 linear_velocity{ reference_motion.vx, reference_motion.vy,
                                                     reference_motion.vz };
    const sensor_interface::Vector3 angular_velocity{ reference_motion.wx, reference_motion.wy,
                                                      reference_motion.wz };
    transform3d = sensor_interface::expTransform3D(linear_velocity, angular_velocity, dt);
  }
  else
  {
    sensor_interface::integrateTwist2D(reference_motion.vx, reference_motion.vy,
                                       reference_motion.wz, dt, &dx, &dy, &dyaw);
    cos_yaw = std::cos(dyaw);
    sin_yaw = std::sin(dyaw);
  }

  for (size_t i = 0; i < point_count; ++i)
  {
    const size_t point_offset = output_start + i * static_cast<size_t>(cloud.point_step);
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!sensor_interface::readFieldAsDouble(*output, *x_field, point_offset, &x) ||
        !sensor_interface::readFieldAsDouble(*output, *y_field, point_offset, &y) ||
        !sensor_interface::readFieldAsDouble(*output, *z_field, point_offset, &z))
    {
      continue;
    }
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    {
      continue;
    }

    const tf2::Vector3 transformed = cloud_tf * tf2::Vector3(x, y, z);
    sensor_interface::Point3 point{ transformed.x(), transformed.y(), transformed.z() };
    if (correction_mode_ == "3d")
    {
      point = sensor_interface::transformPoint(transform3d, point);
    }
    else
    {
      point = sensor_interface::Point3{ cos_yaw * point.x - sin_yaw * point.y + dx,
                                        sin_yaw * point.x + cos_yaw * point.y + dy,
                                        point.z };
    }

    sensor_interface::writeFloat32Field(output, *x_field, point_offset, point.x);
    sensor_interface::writeFloat32Field(output, *y_field, point_offset, point.y);
    sensor_interface::writeFloat32Field(output, *z_field, point_offset, point.z);
  }

  return true;
}

void LidarAdvancedConcatFilter::cloud1Callback(const PointCloudMsgT::ConstPtr &msg)
{
  latest_cloud1_ = msg;
  tryPublish();
}

void LidarAdvancedConcatFilter::cloud2Callback(const PointCloudMsgT::ConstPtr &msg)
{
  latest_cloud2_ = msg;
  tryPublish();
}

void LidarAdvancedConcatFilter::tryPublish()
{
  if (!latest_cloud1_ || !latest_cloud2_)
  {
    return;
  }

  const double stamp_delta = std::abs((latest_cloud2_->header.stamp - latest_cloud1_->header.stamp).toSec());
  if (stamp_delta > max_sync_interval_sec_)
  {
    ROS_DEBUG_THROTTLE(1.0, "Waiting for synchronized LiDAR clouds: dt=%.6f exceeds %.6f",
                       stamp_delta, max_sync_interval_sec_);
    return;
  }

  if (latest_cloud1_->header.stamp == last_published_stamp1_ &&
      latest_cloud2_->header.stamp == last_published_stamp2_)
  {
    return;
  }

  last_published_stamp1_ = latest_cloud1_->header.stamp;
  last_published_stamp2_ = latest_cloud2_->header.stamp;
  publishPair(latest_cloud1_, latest_cloud2_);
}

void LidarAdvancedConcatFilter::publishPair(const PointCloudMsgT::ConstPtr &msg1,
                                            const PointCloudMsgT::ConstPtr &msg2)
{
  if (!sensor_interface::samePointSchema(*msg1, *msg2))
  {
    ROS_WARN_THROTTLE(1.0, "Input clouds have different PointCloud2 schemas; refusing field-unsafe concat");
    return;
  }

  const ros::Time reference_stamp = msg1->header.stamp <= msg2->header.stamp ? msg1->header.stamp : msg2->header.stamp;

  sensor_interface::MotionSample reference_motion;
  if (enable_motion_compensation_ && !getReferenceMotion(reference_stamp, &reference_motion))
  {
    return;
  }

  PointCloudMsgT output;
  output.header.stamp = reference_stamp;
  output.header.frame_id = output_frame_id_;
  output.height = 1;
  output.width = static_cast<uint32_t>(static_cast<size_t>(msg1->width) * static_cast<size_t>(msg1->height) +
                                       static_cast<size_t>(msg2->width) * static_cast<size_t>(msg2->height));
  output.fields = msg1->fields;
  output.is_bigendian = msg1->is_bigendian;
  output.point_step = msg1->point_step;
  output.row_step = output.point_step * output.width;
  output.is_dense = msg1->is_dense && msg2->is_dense;
  output.data.reserve(static_cast<size_t>(output.row_step));

  if (!appendCompensatedCloud(*msg1, reference_stamp, reference_motion, &output))
  {
    return;
  }
  if (!appendCompensatedCloud(*msg2, reference_stamp, reference_motion, &output))
  {
    return;
  }

  output.width = static_cast<uint32_t>(output.data.size() / output.point_step);
  output.row_step = output.point_step * output.width;
  cloud_publisher_.publish(output);

  ROS_DEBUG_STREAM_THROTTLE(1.0, "advanced concat published " << output.width << " points dt12="
                                                              << (msg2->header.stamp - msg1->header.stamp).toSec()
                                                              << " mode=" << correction_mode_
                                                              << " vx=" << reference_motion.vx
                                                              << " vy=" << reference_motion.vy
                                                              << " vz=" << reference_motion.vz
                                                              << " wx=" << reference_motion.wx
                                                              << " wy=" << reference_motion.wy
                                                              << " wz=" << reference_motion.wz);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_advanced_concat_filter");
  LidarAdvancedConcatFilter node;
  ros::spin();
  return 0;
}
