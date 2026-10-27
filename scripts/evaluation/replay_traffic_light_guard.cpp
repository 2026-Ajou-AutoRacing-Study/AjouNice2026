// Read-only stdin adapter to the SOURCE pure Guard, never a ROS/planner node.
// Input: id key required grace phase source_ns now_ns epoch_ns color reset timeout
// Integer ns are subtracted before conversion, preserving sub-ms epoch timing.
#include <autoware/behavior_velocity_traffic_light_module/signal_compliance_guard.hpp>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

namespace sc = autoware::behavior_velocity_planner::signal_compliance;

int main()
{
  std::map<std::string, sc::Guard> guards;
  std::string line;
  std::cout << "id,require_stop,green_count,grace_active,yellow_eligible,yellow_lower_sec\n";
  while (std::getline(std::cin, line)) {
    std::istringstream row(line);
    std::uint64_t id;
    std::string key, phase;
    std::size_t required;
    double grace, timeout;
    std::int64_t source_ns, now_ns, epoch_ns;
    int color, reset;
    if (!(row >> id >> key >> required >> grace >> phase >> source_ns >> now_ns >>
          epoch_ns >> color >> reset >> timeout)) return 2;
    sc::Observation observation;
    if (phase == "green") observation = sc::Observation::AuthorizedGreen;
    else if (phase == "yellow") observation = sc::Observation::Yellow;
    else if (phase == "stop") observation = sc::Observation::Stop;
    else if (phase == "unknown") observation = sc::Observation::Unknown;
    else if (phase == "missing") observation = sc::Observation::Missing;
    else if (phase == "stale") observation = sc::Observation::Stale;
    else return 3;
    auto & guard = guards[key];
    guard.setRequiredGreenObservations(required);
    guard.setUnknownGraceSec(grace);
    if (reset) guard.reset();
    const auto seconds = [epoch_ns](std::int64_t ns) {
      return ns > 0 ? static_cast<double>(ns - epoch_ns) / 1e9 : 0.0;
    };
    const bool stop = guard.observe(observation, seconds(source_ns), seconds(now_ns),
                                    timeout, color != 0);
    std::cout << id << ',' << stop << ',' << guard.greenConfirmationCount() << ',' <<
      guard.unknownGraceActive() << ',' << guard.yellowTransitionEligible() << ',';
    if (guard.yellowStartLowerBoundSec()) {
      std::cout << std::setprecision(17) << *guard.yellowStartLowerBoundSec();
    }
    std::cout << '\n';
  }
}
