#include <cmath>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <morai_msgs/GPSMessage.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84InvF = 298.257223563;
constexpr double kUtmScale = 0.9996;
constexpr double kDegToRad = M_PI / 180.0;

struct UtmPoint
{
  double easting = 0.0;
  double northing = 0.0;
};

bool is_finite(double value)
{
  return std::isfinite(value);
}

UtmPoint wgs84_to_utm(double latitude_deg, double longitude_deg, int zone, bool northern_hemisphere)
{
  const double f = 1.0 / kWgs84InvF;
  const double e2 = f * (2.0 - f);
  const double ep2 = e2 / (1.0 - e2);

  const double latitude = latitude_deg * kDegToRad;
  const double longitude = longitude_deg * kDegToRad;
  const double central_meridian = ((zone - 1) * 6 - 180 + 3) * kDegToRad;

  const double sin_lat = std::sin(latitude);
  const double cos_lat = std::cos(latitude);
  const double tan_lat = std::tan(latitude);

  const double n = kWgs84A / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
  const double t = tan_lat * tan_lat;
  const double c = ep2 * cos_lat * cos_lat;
  const double a = cos_lat * (longitude - central_meridian);

  const double m = kWgs84A *
                   ((1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0) * latitude -
                    (3.0 * e2 / 8.0 + 3.0 * e2 * e2 / 32.0 + 45.0 * e2 * e2 * e2 / 1024.0) *
                        std::sin(2.0 * latitude) +
                    (15.0 * e2 * e2 / 256.0 + 45.0 * e2 * e2 * e2 / 1024.0) * std::sin(4.0 * latitude) -
                    (35.0 * e2 * e2 * e2 / 3072.0) * std::sin(6.0 * latitude));

  UtmPoint point;
  point.easting = kUtmScale * n *
                      (a + (1.0 - t + c) * std::pow(a, 3.0) / 6.0 +
                       (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * ep2) * std::pow(a, 5.0) / 120.0) +
                  500000.0;
  point.northing =
      kUtmScale *
      (m + n * tan_lat *
               (a * a / 2.0 + (5.0 - t + 9.0 * c + 4.0 * c * c) * std::pow(a, 4.0) / 24.0 +
                (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * ep2) * std::pow(a, 6.0) / 720.0));

  if (!northern_hemisphere)
  {
    point.northing += 10000000.0;
  }

  return point;
}

geometry_msgs::PoseStamped apply_transform(const geometry_msgs::PoseStamped &pose,
                                           const geometry_msgs::TransformStamped &transform,
                                           const std::string &target_frame)
{
  tf2::Quaternion rotation;
  tf2::fromMsg(transform.transform.rotation, rotation);
  rotation.normalize();

  const tf2::Vector3 translation(transform.transform.translation.x,
                                 transform.transform.translation.y,
                                 transform.transform.translation.z);
  const tf2::Vector3 point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  const tf2::Vector3 transformed_point = tf2::Matrix3x3(rotation) * point + translation;

  tf2::Quaternion pose_rotation;
  tf2::fromMsg(pose.pose.orientation, pose_rotation);
  pose_rotation.normalize();

  tf2::Quaternion transformed_rotation = rotation * pose_rotation;
  transformed_rotation.normalize();

  geometry_msgs::PoseStamped output;
  output.header.stamp = pose.header.stamp;
  output.header.frame_id = target_frame;
  output.pose.position.x = transformed_point.x();
  output.pose.position.y = transformed_point.y();
  output.pose.position.z = transformed_point.z();
  output.pose.orientation = tf2::toMsg(transformed_rotation);
  return output;
}

}  // namespace

class MoraiGpsToMap
{
public:
  MoraiGpsToMap()
    : private_node_handle_("~")
    , tf_listener_(tf_buffer_)
  {
    private_node_handle_.param("input_topic", input_topic_, std::string("/sensor/gps"));
    private_node_handle_.param("output_pose_topic", output_pose_topic_, std::string("/sensor/gps/map_pose"));
    private_node_handle_.param("output_morai_pose_topic",
                               output_morai_pose_topic_,
                               std::string("/sensor/gps/morai_pose"));

    private_node_handle_.param("map_frame_id", map_frame_id_, std::string("map"));
    private_node_handle_.param("morai_frame_id", morai_frame_id_, std::string("morai_map"));

    private_node_handle_.param("utm_zone", utm_zone_, 52);
    private_node_handle_.param("northern_hemisphere", northern_hemisphere_, true);
    private_node_handle_.param("use_message_offsets", use_message_offsets_, true);
    private_node_handle_.param("fallback_east_offset", fallback_east_offset_, 302595.0);
    private_node_handle_.param("fallback_north_offset", fallback_north_offset_, 4124145.0);
    private_node_handle_.param("morai_origin_altitude", morai_origin_altitude_, 0.0);

    private_node_handle_.param("require_status_ok", require_status_ok_, true);
    private_node_handle_.param("publish_morai_pose", publish_morai_pose_, true);
    private_node_handle_.param("tf_timeout_sec", tf_timeout_sec_, 0.1);

    pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(output_pose_topic_, 10);
    morai_pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(output_morai_pose_topic_, 10);
    gps_subscriber_ = node_handle_.subscribe(input_topic_, 50, &MoraiGpsToMap::gps_callback, this);

    ROS_INFO_STREAM("morai_gps_to_map: " << input_topic_ << " -> " << output_pose_topic_
                                         << " using UTM zone " << utm_zone_
                                         << (northern_hemisphere_ ? "N" : "S") << " and TF "
                                         << morai_frame_id_ << " -> " << map_frame_id_);
  }

private:
  void gps_callback(const morai_msgs::GPSMessage::ConstPtr &msg)
  {
    if (require_status_ok_ && msg->status <= 0)
    {
      ROS_WARN_THROTTLE(2.0, "Skipping GPS message with invalid status=%d", msg->status);
      return;
    }

    if (!is_finite(msg->latitude) || !is_finite(msg->longitude) || !is_finite(msg->altitude))
    {
      ROS_WARN_THROTTLE(2.0, "Skipping GPS message with non-finite lat/lon/alt");
      return;
    }

    const double east_offset = choose_offset(msg->eastOffset, fallback_east_offset_);
    const double north_offset = choose_offset(msg->northOffset, fallback_north_offset_);
    const UtmPoint utm = wgs84_to_utm(msg->latitude, msg->longitude, utm_zone_, northern_hemisphere_);

    geometry_msgs::PoseStamped morai_pose;
    morai_pose.header.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    morai_pose.header.frame_id = morai_frame_id_;
    morai_pose.pose.position.x = utm.easting - east_offset;
    morai_pose.pose.position.y = utm.northing - north_offset;
    morai_pose.pose.position.z = msg->altitude - morai_origin_altitude_;
    morai_pose.pose.orientation.w = 1.0;

    geometry_msgs::TransformStamped map_from_morai;
    try
    {
      map_from_morai =
          tf_buffer_.lookupTransform(map_frame_id_, morai_frame_id_, ros::Time(0), ros::Duration(tf_timeout_sec_));
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_WARN_THROTTLE(2.0,
                        "Waiting for TF %s -> %s: %s",
                        morai_frame_id_.c_str(),
                        map_frame_id_.c_str(),
                        ex.what());
      return;
    }

    const geometry_msgs::PoseStamped map_pose = apply_transform(morai_pose, map_from_morai, map_frame_id_);

    if (publish_morai_pose_)
    {
      morai_pose_publisher_.publish(morai_pose);
    }

    pose_publisher_.publish(map_pose);
  }

  double choose_offset(double message_offset, double fallback_offset) const
  {
    if (use_message_offsets_ && is_finite(message_offset) && std::fabs(message_offset) > 1.0)
    {
      return message_offset;
    }
    return fallback_offset;
  }

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber gps_subscriber_;
  ros::Publisher pose_publisher_;
  ros::Publisher morai_pose_publisher_;

  std::string input_topic_;
  std::string output_pose_topic_;
  std::string output_morai_pose_topic_;
  std::string map_frame_id_;
  std::string morai_frame_id_;

  int utm_zone_ = 52;
  bool northern_hemisphere_ = true;
  bool use_message_offsets_ = true;
  double fallback_east_offset_ = 302595.0;
  double fallback_north_offset_ = 4124145.0;
  double morai_origin_altitude_ = 0.0;
  bool require_status_ok_ = true;
  bool publish_morai_pose_ = true;
  double tf_timeout_sec_ = 0.1;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "morai_gps_to_map");
  MoraiGpsToMap node;
  ros::spin();
  return 0;
}
