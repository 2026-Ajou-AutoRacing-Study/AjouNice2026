// Phase2-authorized ROS1 cardinality adapter for InitializeLocalization.
#ifndef AUTOWARE_LOCALIZATION_MSGS__INITIALIZE_LOCALIZATION_BOUND_ADAPTER_HPP_
#define AUTOWARE_LOCALIZATION_MSGS__INITIALIZE_LOCALIZATION_BOUND_ADAPTER_HPP_

#include <autoware_localization_msgs/InitializeLocalization.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <stdexcept>
#include <vector>

namespace autoware_localization_msgs
{
namespace bounded_sequence
{
constexpr std::size_t kInitializeLocalizationPoseMaxSize = 1U;

inline void validate_initialize_localization_request(
  const autoware_localization_msgs::InitializeLocalizationRequest & request)
{
  if (request.pose_with_covariance.size() > kInitializeLocalizationPoseMaxSize) {
    throw std::length_error(
      "InitializeLocalization.pose_with_covariance exceeds ROS2 bound <=1");
  }
}

inline void clear_initialize_localization_pose(
  autoware_localization_msgs::InitializeLocalizationRequest & request)
{
  request.pose_with_covariance.clear();
  validate_initialize_localization_request(request);
}

inline void set_initialize_localization_pose(
  autoware_localization_msgs::InitializeLocalizationRequest & request,
  const geometry_msgs::PoseWithCovarianceStamped & pose)
{
  request.pose_with_covariance.assign(1U, pose);
  validate_initialize_localization_request(request);
}

inline const std::vector<geometry_msgs::PoseWithCovarianceStamped> &
validated_initialize_localization_poses(
  const autoware_localization_msgs::InitializeLocalizationRequest & request)
{
  validate_initialize_localization_request(request);
  return request.pose_with_covariance;
}

template<class RosServiceClient>
inline bool call_initialize_localization_checked(
  RosServiceClient & client, autoware_localization_msgs::InitializeLocalization & service)
{
  // The only authorized transport call: validation is structurally before transport.
  validate_initialize_localization_request(service.request);
  return client.call(service);
}

class ValidatedInitializeLocalizationRequestView
{
public:
  const autoware_localization_msgs::InitializeLocalizationRequest & get() const {return request_;}
private:
  explicit ValidatedInitializeLocalizationRequestView(
    const autoware_localization_msgs::InitializeLocalizationRequest & request) : request_(request) {}
  const autoware_localization_msgs::InitializeLocalizationRequest & request_;
  template<class Handler> friend class InitializeLocalizationCheckedHandler;
};

template<class Handler>
class InitializeLocalizationCheckedHandler
{
public:
  explicit InitializeLocalizationCheckedHandler(Handler handler) : handler_(handler) {}
  bool operator()(
    autoware_localization_msgs::InitializeLocalizationRequest & request,
    autoware_localization_msgs::InitializeLocalizationResponse & response)
  {
    // The raw request cannot reach domain code until validation succeeds.
    validate_initialize_localization_request(request);
    return handler_(ValidatedInitializeLocalizationRequestView(request), response);
  }
private:
  Handler handler_;
};

template<class Handler>
inline InitializeLocalizationCheckedHandler<Handler> make_initialize_localization_checked_handler(
  Handler handler)
{
  return InitializeLocalizationCheckedHandler<Handler>(handler);
}
}  // namespace bounded_sequence
}  // namespace autoware_localization_msgs

#endif
