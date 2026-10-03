#pragma once
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>
namespace autoware::behavior_path_planner::lane_change::rss_positive_grid {
inline std::vector<double> augment(std::vector<double> targets,double maximum,bool enabled) {
 if(!enabled)return targets;
 if(!std::isfinite(maximum)||maximum<0)throw std::logic_error("invalid positive grid maximum");
 for(double factor:{.75,.25,.1}) {
   const double value=maximum*factor;
   if(value>0&&std::find(targets.begin(),targets.end(),value)==targets.end())targets.push_back(value);
 }
 std::stable_sort(targets.begin(),targets.end(),std::greater<double>());
 if(targets.size()>15)throw std::logic_error("positive grid family exceeds budget");
 return targets;
}
}
