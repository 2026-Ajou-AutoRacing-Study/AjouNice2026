#include "autoware/rtc_interface/rtc_interface.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace
{
using autoware::rtc_interface::RTCInterface;

// behavior_path_manager_shared_map: live owner NodeHandle outlives shared map values.
struct BehaviorPathManagerSharedMap {
  explicit BehaviorPathManagerSharedMap(ros::NodeHandle & node)
  : node_(node), rtc_(std::make_shared<RTCInterface>(&node_, "behavior_path", true)) {}
  ros::NodeHandle node_;
  std::shared_ptr<RTCInterface> rtc_;
};

// velocity_rtc_normal_embedded: embedded RTCInterface follows its live owner handle.
struct VelocityRtcNormalEmbedded {
  explicit VelocityRtcNormalEmbedded(ros::NodeHandle & node) : rtc_(&node, "velocity_normal", true) {}
  RTCInterface rtc_;
};

// velocity_rtc_experimental_embedded: experimental embedded owner has the same boundary.
struct VelocityRtcExperimentalEmbedded {
  explicit VelocityRtcExperimentalEmbedded(ros::NodeHandle & node)
  : rtc_(&node, "velocity_experimental", true) {}
  RTCInterface rtc_;
};

// intersection_normal_occlusion_embedded: normal occlusion member keeps node external.
struct IntersectionNormalOcclusionEmbedded {
  explicit IntersectionNormalOcclusionEmbedded(ros::NodeHandle & node)
  : rtc_(&node, "intersection_occlusion", true) {}
  RTCInterface rtc_;
};

// intersection_experimental_occlusion_embedded: experimental owner keeps the same lifetime order.
struct IntersectionExperimentalOcclusionEmbedded {
  explicit IntersectionExperimentalOcclusionEmbedded(ros::NodeHandle & node)
  : rtc_(&node, "intersection_occlusion", true) {}
  RTCInterface rtc_;
};

TEST(RTCInterfaceOwnerContracts, all_five_constructor_owner_overlays_compile_and_destroy_after_dispatch_stops)
{
  if (!ros::isInitialized()) {
    int argc = 0;
    char ** argv = nullptr;
    ros::init(argc, argv, "rtc_interface_owner_contracts", ros::init_options::AnonymousName);
  }
  ros::NodeHandle node;
  { BehaviorPathManagerSharedMap owner(node); }
  { VelocityRtcNormalEmbedded owner(node); }
  { VelocityRtcExperimentalEmbedded owner(node); }
  { IntersectionNormalOcclusionEmbedded owner(node); }
  { IntersectionExperimentalOcclusionEmbedded owner(node); }
  SUCCEED();
}
}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
