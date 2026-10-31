// Offline trace through the production pure observation Guard; no ROS/planner.
#include <autoware/behavior_velocity_traffic_light_module/signal_compliance_guard.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <utility>

int main(int argc,char ** argv)
{
  if(argc!=2)return 2;
  std::ifstream input(argv[1]);
  if(!input)return 2;
  using autoware::behavior_velocity_planner::signal_compliance::Guard;
  using autoware::behavior_velocity_planner::signal_compliance::Observation;
  std::map<std::pair<std::string,long>,Guard> guards;
  std::string line;
  std::cout << "t,mode,section,group_id,require_stop,green_count\n";
  while(std::getline(input,line)) {
    std::istringstream row(line);double t,source,now;std::string mode,section,phase;long group;
    if(!(row>>t>>mode>>section>>group>>source>>now>>phase))return 3;
    const auto observation=phase=="green"?Observation::AuthorizedGreen:
      phase=="yellow"?Observation::Yellow:phase=="red"?Observation::Stop:Observation::Unknown;
    auto & guard=guards[{mode,group}];
    const bool stop=guard.observe(observation,source,now,1.);
    std::cout<<std::setprecision(15)<<t<<','<<mode<<','<<section<<','<<group<<','<<stop<<','
      <<guard.greenConfirmationCount()<<'\n';
  }
  return 0;
}
