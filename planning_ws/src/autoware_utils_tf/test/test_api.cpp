#include <autoware_utils_tf/self_pose_listener.hpp>
#include <autoware_utils_tf/transform_listener.hpp>
#include <gtest/gtest.h>
#include <type_traits>

namespace
{
using TfListener = autoware_utils_tf::TransformListener;
using PoseListener = autoware_utils_tf::SelfPoseListener;
static_assert(std::is_constructible<TfListener, const ros::NodeHandle &>::value, "NodeHandle constructor required");
static_assert(std::is_constructible<PoseListener, const ros::NodeHandle &>::value, "NodeHandle constructor required");
static_assert(!std::is_default_constructible<TfListener>::value, "no implicit default listener");
static_assert(!std::is_default_constructible<PoseListener>::value, "no implicit default pose listener");
static_assert(std::is_same<decltype(std::declval<const TfListener &>().get_latest_transform(std::declval<const std::string &>(), std::declval<const std::string &>())), geometry_msgs::TransformStamped::ConstPtr>::value, "latest signature");
static_assert(std::is_same<decltype(std::declval<const TfListener &>().get_transform(std::declval<const std::string &>(), std::declval<const std::string &>(), std::declval<const ros::Time &>(), std::declval<const ros::Duration &>())), geometry_msgs::TransformStamped::ConstPtr>::value, "timed signature");
static_assert(std::is_same<decltype(std::declval<const PoseListener &>().get_current_pose()), geometry_msgs::PoseStamped::ConstPtr>::value, "pose signature");
TEST(ApiContract, IsCxx17AndRos1Typed) { SUCCEED(); }
}  // namespace
