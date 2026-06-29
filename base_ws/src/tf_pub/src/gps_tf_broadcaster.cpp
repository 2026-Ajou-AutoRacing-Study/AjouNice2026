#include <cmath>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <morai_msgs/GPSMessage.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace
{
constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_F = 1.0 / 298.257223563;
constexpr double UTM_K0 = 0.9996;

bool isFiniteGps(const morai_msgs::GPSMessage &msg)
{
  return std::isfinite(msg.latitude) && std::isfinite(msg.longitude) && std::isfinite(msg.altitude) &&
         std::isfinite(msg.eastOffset) && std::isfinite(msg.northOffset);
}

double chooseOffset(double message_offset, double fallback_offset, bool use_message_offsets)
{
  if (use_message_offsets && std::isfinite(message_offset) && std::abs(message_offset) > 1.0)
  {
    return message_offset;
  }
  return fallback_offset;
}

void wgs84ToUtm(double latitude_deg, double longitude_deg, int zone, double *easting, double *northing)
{
  const double latitude = latitude_deg * M_PI / 180.0;
  const double longitude = longitude_deg * M_PI / 180.0;
  const double central_meridian = ((zone - 1) * 6 - 180 + 3) * M_PI / 180.0;

  const double e2 = WGS84_F * (2.0 - WGS84_F);
  const double ep2 = e2 / (1.0 - e2);
  const double sin_lat = std::sin(latitude);
  const double cos_lat = std::cos(latitude);
  const double tan_lat = std::tan(latitude);

  const double n = WGS84_A / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
  const double t = tan_lat * tan_lat;
  const double c = ep2 * cos_lat * cos_lat;
  const double a = cos_lat * (longitude - central_meridian);

  const double m =
      WGS84_A * ((1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0) * latitude -
                 (3.0 * e2 / 8.0 + 3.0 * e2 * e2 / 32.0 + 45.0 * e2 * e2 * e2 / 1024.0) *
                     std::sin(2.0 * latitude) +
                 (15.0 * e2 * e2 / 256.0 + 45.0 * e2 * e2 * e2 / 1024.0) * std::sin(4.0 * latitude) -
                 (35.0 * e2 * e2 * e2 / 3072.0) * std::sin(6.0 * latitude));

  *easting = UTM_K0 * n *
                 (a + (1.0 - t + c) * std::pow(a, 3.0) / 6.0 +
                  (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * ep2) * std::pow(a, 5.0) / 120.0) +
             500000.0;

  *northing = UTM_K0 *
              (m + n * tan_lat *
                       (a * a / 2.0 + (5.0 - t + 9.0 * c + 4.0 * c * c) * std::pow(a, 4.0) / 24.0 +
                        (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * ep2) * std::pow(a, 6.0) / 720.0));

  if (latitude_deg < 0.0)
  {
    *northing += 10000000.0;
  }
}
}  // namespace

class GpsTfBroadcaster
{
public:
  GpsTfBroadcaster()
  {
    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("gps_topic", gps_topic_, "/sensor/gps");
    private_nh.param<std::string>("parent_frame", parent_frame_, "morai_map");
    private_nh.param<std::string>("child_frame", child_frame_, "base_link");
    private_nh.param<int>("utm_zone", utm_zone_, 52);
    private_nh.param<bool>("use_message_offsets", use_message_offsets_, true);
    private_nh.param<double>("fallback_east_offset", fallback_east_offset_, 302595.0);
    private_nh.param<double>("fallback_north_offset", fallback_north_offset_, 4124145.0);
    private_nh.param<double>("morai_origin_altitude", morai_origin_altitude_, 0.0);
    private_nh.param<double>("gps_sensor_z_offset", gps_sensor_z_offset_, 1.23);
    private_nh.param<double>("min_heading_distance", min_heading_distance_, 0.05);

    gps_subscriber_ = node_handle_.subscribe(gps_topic_, 50, &GpsTfBroadcaster::gpsCallback, this);

    ROS_INFO_STREAM("gps_tf_broadcaster subscribe: " << gps_topic_);
    ROS_INFO_STREAM("gps_tf_broadcaster publish TF: " << parent_frame_ << " -> " << child_frame_);
  }

private:
  void gpsCallback(const morai_msgs::GPSMessage::ConstPtr &msg)
  {
    if (msg->status <= 0)
    {
      ROS_WARN_THROTTLE(2.0, "Skipping GPS TF with invalid status=%d", msg->status);
      return;
    }
    if (!isFiniteGps(*msg))
    {
      ROS_WARN_THROTTLE(2.0, "Skipping GPS TF with non-finite lat/lon/alt/offset");
      return;
    }

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    if (stamp == last_stamp_)
    {
      return;
    }
    last_stamp_ = stamp;

    double easting = 0.0;
    double northing = 0.0;
    wgs84ToUtm(msg->latitude, msg->longitude, utm_zone_, &easting, &northing);

    const double east_offset = chooseOffset(msg->eastOffset, fallback_east_offset_, use_message_offsets_);
    const double north_offset = chooseOffset(msg->northOffset, fallback_north_offset_, use_message_offsets_);
    const double x = easting - east_offset;
    const double y = northing - north_offset;
    const double z = msg->altitude - morai_origin_altitude_ - gps_sensor_z_offset_;

    if (has_last_xy_)
    {
      const double dx = x - last_x_;
      const double dy = y - last_y_;
      if (std::hypot(dx, dy) >= min_heading_distance_)
      {
        yaw_ = std::atan2(dy, dx);
      }
    }
    last_x_ = x;
    last_y_ = y;
    has_last_xy_ = true;

    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, yaw_);
    rotation.normalize();

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = child_frame_;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();

    tf_broadcaster_.sendTransform(transform);
  }

  ros::NodeHandle node_handle_;
  ros::Subscriber gps_subscriber_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string gps_topic_;
  std::string parent_frame_;
  std::string child_frame_;
  int utm_zone_ = 52;
  bool use_message_offsets_ = true;
  double fallback_east_offset_ = 302595.0;
  double fallback_north_offset_ = 4124145.0;
  double morai_origin_altitude_ = 0.0;
  double gps_sensor_z_offset_ = 1.23;
  double min_heading_distance_ = 0.05;

  ros::Time last_stamp_;
  bool has_last_xy_ = false;
  double last_x_ = 0.0;
  double last_y_ = 0.0;
  double yaw_ = 0.0;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_tf_broadcaster");
  GpsTfBroadcaster broadcaster;
  ros::spin();
  return 0;
}
