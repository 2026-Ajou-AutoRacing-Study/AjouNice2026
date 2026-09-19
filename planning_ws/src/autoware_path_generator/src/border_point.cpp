#include <autoware/path_generator/border_point.hpp>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/utility/Utilities.h>
#include <cmath>
namespace autoware::path_generator::utils {
std::optional<lanelet::ConstPoint3d> get_border_point(const lanelet::BasicLineString3d & segment, const lanelet::ConstLanelet & lanelet) {
  if (segment.size() < 2) return std::nullopt;
  const lanelet::BasicLineString3d border{lanelet.leftBound().front().basicPoint(), lanelet.rightBound().front().basicPoint()};
  const auto cross=[](const auto & a,const auto & b,const auto & c){return (b.x()-a.x())*(c.y()-a.y())-(b.y()-a.y())*(c.x()-a.x());};
  for(size_t i=0;i+1<segment.size();++i){
    const auto & a=segment[i]; const auto & b=segment[i+1]; const auto & c=border.front(); const auto & d=border.back();
    const double denom=(b.x()-a.x())*(d.y()-c.y())-(b.y()-a.y())*(d.x()-c.x());
    if(std::abs(denom)<1e-12) continue;
    const double t=cross(c,d,a)/denom, u=cross(a,b,c)/-denom;
    if(t>=0.&&t<=1.&&u>=0.&&u<=1.) return lanelet::utils::to3D(lanelet::ConstPoint2d(lanelet::InvalId,{a.x()+t*(b.x()-a.x()),a.y()+t*(b.y()-a.y()),segment.back().z()}));
  }
  return std::nullopt;
}
bool is_pose_inside_lanelets(const geometry_msgs::Pose & pose, const lanelet::ConstLanelets & lanelets) {
  const lanelet::BasicPoint2d point{pose.position.x, pose.position.y};
  for (const auto & lanelet : lanelets) {
    if (lanelet::geometry::inside(lanelet, point)) return true;
  }
  return false;
}
}
