#include <gtest/gtest.h>

#include <autoware/universe_utils/geometry/alt_geometry.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware/universe_utils/geometry/ear_clipping.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/geometry/gjk_2d.hpp>
#include <autoware/universe_utils/geometry/pose_deviation.hpp>
#include <autoware/universe_utils/geometry/polygon_clip.hpp>
#include <autoware/universe_utils/geometry/random_concave_polygon.hpp>
#include <autoware/universe_utils/geometry/random_convex_polygon.hpp>
#include <autoware/universe_utils/geometry/sat_2d.hpp>
#include <autoware/universe_utils/math/accumulator.hpp>
#include <autoware/universe_utils/math/constants.hpp>
#include <autoware/universe_utils/math/normalization.hpp>
#include <autoware/universe_utils/math/range.hpp>
#include <autoware/universe_utils/math/sin_table.hpp>
#include <autoware/universe_utils/math/trigonometry.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>
#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware/universe_utils/ros/debug_traits.hpp>
#include <autoware/universe_utils/ros/diagnostics_interface.hpp>
#include <autoware/universe_utils/ros/logger_level_configure.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <autoware/universe_utils/ros/msg_covariance.hpp>
#include <autoware/universe_utils/ros/msg_operation.hpp>
#include <autoware/universe_utils/ros/parameter.hpp>
#include <autoware/universe_utils/ros/pcl_conversion.hpp>
#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include <autoware/universe_utils/ros/processing_time_publisher.hpp>
#include <autoware/universe_utils/ros/published_time_publisher.hpp>
#include <autoware/universe_utils/ros/self_pose_listener.hpp>
#include <autoware/universe_utils/ros/transform_listener.hpp>
#include <autoware/universe_utils/ros/update_param.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>
#include <autoware/universe_utils/ros/wait_for_param.hpp>
#include <autoware/universe_utils/system/backtrace.hpp>
#include <autoware/universe_utils/system/lru_cache.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <autoware/universe_utils/system/time_keeper.hpp>
#include <autoware/universe_utils/transform/transforms.hpp>

TEST(AutowareUniverseUtilsFacade, PreservesCoreUtilityNames)
{
  autoware::universe_utils::LRUCache<int, int> cache(2);
  cache.put(1, 10);
  EXPECT_EQ(cache.get(1), 10);
  EXPECT_DOUBLE_EQ(autoware::universe_utils::deg2rad(180.0), autoware::universe_utils::pi);

  autoware::universe_utils::ProcessingTimeNode root("root");
  root.set_time(1.5);
  EXPECT_EQ(root.get_name(), "root");
  EXPECT_NE(root.to_string().find("root"), std::string::npos);

  const autoware::universe_utils::Point2d a(0.0, 0.0);
  const autoware::universe_utils::Point2d b(1.0, 1.0);
  const autoware::universe_utils::Point2d c(0.0, 1.0);
  const autoware::universe_utils::Point2d d(1.0, 0.0);
  EXPECT_EQ(autoware::universe_utils::intersection(a, b, c, d).size(), 1U);
}
