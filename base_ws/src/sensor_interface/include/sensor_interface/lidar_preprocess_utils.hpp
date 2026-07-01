#ifndef SENSOR_INTERFACE_LIDAR_PREPROCESS_UTILS_HPP_
#define SENSOR_INTERFACE_LIDAR_PREPROCESS_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <string>

#include <Eigen/Dense>
#include <geometry_msgs/TransformStamped.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace sensor_interface
{

struct Point3
{
  double x;
  double y;
  double z;
};

struct Vector3
{
  double x;
  double y;
  double z;
};

struct MotionSample
{
  MotionSample()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , yaw(0.0)
    , vx(0.0)
    , vy(0.0)
    , vz(0.0)
    , wx(0.0)
    , wy(0.0)
    , wz(0.0)
  {
  }

  ros::Time stamp;
  double x;
  double y;
  double z;
  double yaw;
  double vx;
  double vy;
  double vz;
  double wx;
  double wy;
  double wz;
};

struct ImuSample
{
  ImuSample() : wx(0.0), wy(0.0), wz(0.0)
  {
  }

  ros::Time stamp;
  double wx;
  double wy;
  double wz;
};

inline double normalizeAngle(double value)
{
  while (value > M_PI)
  {
    value -= 2.0 * M_PI;
  }
  while (value < -M_PI)
  {
    value += 2.0 * M_PI;
  }
  return value;
}

inline double interpolateAngle(double from, double to, double ratio)
{
  return normalizeAngle(from + normalizeAngle(to - from) * ratio);
}

inline const sensor_msgs::PointField *findField(const sensor_msgs::PointCloud2 &cloud,
                                                const std::string &name)
{
  for (size_t i = 0; i < cloud.fields.size(); ++i)
  {
    if (cloud.fields[i].name == name)
    {
      return &cloud.fields[i];
    }
  }
  return nullptr;
}

inline bool readFieldAsDouble(const sensor_msgs::PointCloud2 &cloud, const sensor_msgs::PointField &field,
                              const size_t point_offset, double *value)
{
  if (value == nullptr || field.count == 0)
  {
    return false;
  }

  const size_t offset = point_offset + field.offset;
  if (offset >= cloud.data.size())
  {
    return false;
  }

  switch (field.datatype)
  {
    case sensor_msgs::PointField::FLOAT32:
    {
      if (offset + sizeof(float) > cloud.data.size())
      {
        return false;
      }
      float raw = 0.0F;
      std::memcpy(&raw, &cloud.data[offset], sizeof(float));
      *value = static_cast<double>(raw);
      return true;
    }
    case sensor_msgs::PointField::FLOAT64:
    {
      if (offset + sizeof(double) > cloud.data.size())
      {
        return false;
      }
      double raw = 0.0;
      std::memcpy(&raw, &cloud.data[offset], sizeof(double));
      *value = raw;
      return true;
    }
    default:
      return false;
  }
}

inline bool writeFloat32Field(sensor_msgs::PointCloud2 *cloud, const sensor_msgs::PointField &field,
                              const size_t point_offset, const double value)
{
  if (cloud == nullptr || field.datatype != sensor_msgs::PointField::FLOAT32 || field.count == 0)
  {
    return false;
  }

  const size_t offset = point_offset + field.offset;
  if (offset + sizeof(float) > cloud->data.size())
  {
    return false;
  }

  const float raw = static_cast<float>(value);
  std::memcpy(&cloud->data[offset], &raw, sizeof(float));
  return true;
}

inline bool samePointSchema(const sensor_msgs::PointCloud2 &lhs, const sensor_msgs::PointCloud2 &rhs)
{
  if (lhs.fields.size() != rhs.fields.size() || lhs.point_step != rhs.point_step ||
      lhs.is_bigendian != rhs.is_bigendian)
  {
    return false;
  }

  for (size_t i = 0; i < lhs.fields.size(); ++i)
  {
    const sensor_msgs::PointField &a = lhs.fields[i];
    const sensor_msgs::PointField &b = rhs.fields[i];
    if (a.name != b.name || a.offset != b.offset || a.datatype != b.datatype || a.count != b.count)
    {
      return false;
    }
  }
  return true;
}

inline Point3 transformPoint(const geometry_msgs::TransformStamped &transform, const Point3 &point)
{
  tf2::Transform tf;
  tf2::fromMsg(transform.transform, tf);
  const tf2::Vector3 transformed = tf * tf2::Vector3(point.x, point.y, point.z);
  return Point3{ transformed.x(), transformed.y(), transformed.z() };
}

inline Eigen::Matrix3d skew(const Vector3 &vector)
{
  Eigen::Matrix3d matrix;
  matrix << 0.0, -vector.z, vector.y, vector.z, 0.0, -vector.x, -vector.y, vector.x, 0.0;
  return matrix;
}

inline Eigen::Matrix4d expTransform3D(const Vector3 &linear_velocity,
                                      const Vector3 &angular_velocity,
                                      const double dt)
{
  const Vector3 translation{ linear_velocity.x * dt, linear_velocity.y * dt,
                             linear_velocity.z * dt };
  const Vector3 rotation{ angular_velocity.x * dt, angular_velocity.y * dt,
                          angular_velocity.z * dt };
  const double theta = std::sqrt(rotation.x * rotation.x + rotation.y * rotation.y +
                                 rotation.z * rotation.z);

  const Eigen::Matrix3d omega = skew(rotation);
  const Eigen::Matrix3d omega2 = omega * omega;
  Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d left_jacobian = Eigen::Matrix3d::Identity();

  if (theta < 1.0e-8)
  {
    rotation_matrix += omega + 0.5 * omega2;
    left_jacobian += 0.5 * omega + (1.0 / 6.0) * omega2;
  }
  else
  {
    const double theta2 = theta * theta;
    const double theta3 = theta2 * theta;
    rotation_matrix += (std::sin(theta) / theta) * omega +
                       ((1.0 - std::cos(theta)) / theta2) * omega2;
    left_jacobian += ((1.0 - std::cos(theta)) / theta2) * omega +
                     ((theta - std::sin(theta)) / theta3) * omega2;
  }

  const Eigen::Vector3d rho(translation.x, translation.y, translation.z);
  const Eigen::Vector3d translated = left_jacobian * rho;

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = rotation_matrix;
  transform.block<3, 1>(0, 3) = translated;
  return transform;
}

inline Point3 transformPoint(const Eigen::Matrix4d &transform, const Point3 &point)
{
  const Eigen::Vector4d input(point.x, point.y, point.z, 1.0);
  const Eigen::Vector4d output = transform * input;
  return Point3{ output.x(), output.y(), output.z() };
}

inline Point3 compensatePoint3D(const Point3 &point, const Vector3 &linear_velocity,
                                const Vector3 &angular_velocity, const double dt)
{
  return transformPoint(expTransform3D(linear_velocity, angular_velocity, dt), point);
}

inline void integrateTwist2D(const double vx, const double vy, const double wz, const double dt,
                             double *dx, double *dy, double *dyaw)
{
  *dyaw = wz * dt;
  if (std::abs(wz) < 1.0e-6)
  {
    *dx = vx * dt;
    *dy = vy * dt;
    return;
  }

  const double sin_yaw = std::sin(*dyaw);
  const double cos_yaw = std::cos(*dyaw);
  *dx = (vx * sin_yaw + vy * (cos_yaw - 1.0)) / wz;
  *dy = (vx * (1.0 - cos_yaw) + vy * sin_yaw) / wz;
}

inline Point3 compensatePoint2D(const Point3 &point, const double vx, const double vy, const double wz,
                                const double dt)
{
  double dx = 0.0;
  double dy = 0.0;
  double dyaw = 0.0;
  integrateTwist2D(vx, vy, wz, dt, &dx, &dy, &dyaw);

  const double cos_yaw = std::cos(dyaw);
  const double sin_yaw = std::sin(dyaw);
  return Point3{ cos_yaw * point.x - sin_yaw * point.y + dx,
                 sin_yaw * point.x + cos_yaw * point.y + dy, point.z };
}

class MotionBuffer
{
public:
  explicit MotionBuffer(const double duration_sec = 2.0) : duration_sec_(duration_sec)
  {
  }

  void setDuration(const double duration_sec)
  {
    duration_sec_ = std::max(0.1, duration_sec);
  }

  void addMotion(const MotionSample &sample)
  {
    if (!motion_samples_.empty() && sample.stamp < motion_samples_.back().stamp)
    {
      motion_samples_.clear();
    }
    motion_samples_.push_back(sample);
    pruneMotion(sample.stamp);
  }

  void addImu(const ImuSample &sample)
  {
    if (!imu_samples_.empty() && sample.stamp < imu_samples_.back().stamp)
    {
      imu_samples_.clear();
    }
    imu_samples_.push_back(sample);
    pruneImu(sample.stamp);
  }

  bool getMotion(const ros::Time &stamp, const double timeout_sec, MotionSample *sample) const
  {
    if (sample == nullptr || motion_samples_.empty())
    {
      return false;
    }

    const auto lower = std::lower_bound(
        motion_samples_.begin(), motion_samples_.end(), stamp,
        [](const MotionSample &lhs, const ros::Time &rhs) { return lhs.stamp < rhs; });

    if (lower == motion_samples_.begin())
    {
      if (std::abs((lower->stamp - stamp).toSec()) <= timeout_sec)
      {
        *sample = *lower;
        sample->stamp = stamp;
        return true;
      }
      return false;
    }

    if (lower == motion_samples_.end())
    {
      const MotionSample &last = motion_samples_.back();
      if (std::abs((stamp - last.stamp).toSec()) <= timeout_sec)
      {
        *sample = last;
        sample->stamp = stamp;
        return true;
      }
      return false;
    }

    const MotionSample &next = *lower;
    const MotionSample &prev = *(lower - 1);
    const double span = (next.stamp - prev.stamp).toSec();
    if (span <= 1.0e-9 || span > std::max(2.0 * timeout_sec, 1.0e-3))
    {
      return false;
    }

    const double ratio = (stamp - prev.stamp).toSec() / span;
    sample->stamp = stamp;
    sample->x = prev.x + (next.x - prev.x) * ratio;
    sample->y = prev.y + (next.y - prev.y) * ratio;
    sample->z = prev.z + (next.z - prev.z) * ratio;
    sample->yaw = interpolateAngle(prev.yaw, next.yaw, ratio);
    sample->vx = prev.vx + (next.vx - prev.vx) * ratio;
    sample->vy = prev.vy + (next.vy - prev.vy) * ratio;
    sample->vz = prev.vz + (next.vz - prev.vz) * ratio;
    sample->wx = prev.wx + (next.wx - prev.wx) * ratio;
    sample->wy = prev.wy + (next.wy - prev.wy) * ratio;
    sample->wz = prev.wz + (next.wz - prev.wz) * ratio;
    return true;
  }

  bool getImu(const ros::Time &stamp, const double timeout_sec, ImuSample *sample) const
  {
    if (sample == nullptr || imu_samples_.empty())
    {
      return false;
    }

    const auto lower = std::lower_bound(
        imu_samples_.begin(), imu_samples_.end(), stamp,
        [](const ImuSample &lhs, const ros::Time &rhs) { return lhs.stamp < rhs; });

    if (lower == imu_samples_.begin())
    {
      if (std::abs((lower->stamp - stamp).toSec()) <= timeout_sec)
      {
        *sample = *lower;
        sample->stamp = stamp;
        return true;
      }
      return false;
    }

    if (lower == imu_samples_.end())
    {
      const ImuSample &last = imu_samples_.back();
      if (std::abs((stamp - last.stamp).toSec()) <= timeout_sec)
      {
        *sample = last;
        sample->stamp = stamp;
        return true;
      }
      return false;
    }

    const ImuSample &next = *lower;
    const ImuSample &prev = *(lower - 1);
    const double span = (next.stamp - prev.stamp).toSec();
    if (span <= 1.0e-9 || span > std::max(2.0 * timeout_sec, 1.0e-3))
    {
      return false;
    }

    const double ratio = (stamp - prev.stamp).toSec() / span;
    sample->stamp = stamp;
    sample->wx = prev.wx + (next.wx - prev.wx) * ratio;
    sample->wy = prev.wy + (next.wy - prev.wy) * ratio;
    sample->wz = prev.wz + (next.wz - prev.wz) * ratio;
    return true;
  }

  bool getImuWz(const ros::Time &stamp, const double timeout_sec, double *wz) const
  {
    ImuSample sample;
    if (wz == nullptr || !getImu(stamp, timeout_sec, &sample))
    {
      return false;
    }
    *wz = sample.wz;
    return true;
  }

private:
  void pruneMotion(const ros::Time &now)
  {
    while (!motion_samples_.empty() && (now - motion_samples_.front().stamp).toSec() > duration_sec_)
    {
      motion_samples_.pop_front();
    }
  }

  void pruneImu(const ros::Time &now)
  {
    while (!imu_samples_.empty() && (now - imu_samples_.front().stamp).toSec() > duration_sec_)
    {
      imu_samples_.pop_front();
    }
  }

  double duration_sec_;
  std::deque<MotionSample> motion_samples_;
  std::deque<ImuSample> imu_samples_;
};

}  // namespace sensor_interface

#endif  // SENSOR_INTERFACE_LIDAR_PREPROCESS_UTILS_HPP_
