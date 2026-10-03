#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>

// Scalar proposal only. The caller must preserve XY/lane IDs and validate all
// final motion hypotheses, raw objects, RSS and vehicle footprints before use.
namespace bounded_speed_grid {
// Bounded heuristic for a hole between a feasible coast and the smallest
// failed positive base sample. Feasibility/RSS need not be monotone: retain
// EVERY successful probe for the caller's full safety validation and ranking.
// No guarantee of a globally feasible/optimal solution is implied.
template<class Generate>
std::vector<double> probe_small_positive(
  bool coast_generated,bool smallest_positive_failed,double upper,Generate generate,size_t max_probes=6)
{
  std::vector<double> attempted;
  if(!coast_generated || !smallest_positive_failed || !std::isfinite(upper) || upper<=0)
    return attempted;
  double lower=0;
  for(size_t n=0;n<std::min(size_t(6),max_probes);++n) {
    const double target=lower+(upper-lower)*.5;
    if(!(target>lower && target<upper))break;
    attempted.push_back(target);
    if(generate(target))lower=target;
    else upper=target;
  }
  return attempted;
}
// Bounded proposal family, not permission to bypass cap/jerk/footprint/RSS
// validation. Retain the original five priorities and fill the moderate-braking
// gap before the separate full-stop fallback.
inline std::vector<double> acceleration_targets(double active_max, double minimum_acceleration)
{
  if (!std::isfinite(active_max) || !std::isfinite(minimum_acceleration) ||
      active_max < 0.0 || minimum_acceleration >= 0.0) {
    throw std::logic_error("invalid acceleration target bounds");
  }
  const std::array<double, 9> nominal{
    active_max, active_max * .5, 0.0, -.5, -1.0, -1.5, -2.0, -3.0,
    minimum_acceleration};
  std::vector<double> targets;
  targets.reserve(nominal.size());
  for (const double value : nominal) {
    const double bounded = std::max(minimum_acceleration, value);
    if (std::find(targets.begin(), targets.end(), bounded) == targets.end()) {
      targets.push_back(bounded);
    }
  }
  return targets;
}
struct State { double t, s, v, a; };
struct Proposal {
  std::vector<double> speeds;
  std::vector<State> samples;
  double progress;
  unsigned tail_attempts=0;
  double tail_factor=1.0;
};
struct RemoteTailConnectionFailure : std::logic_error {
  RemoteTailConnectionFailure():std::logic_error("remote tail cap needs earlier braking after a previous cap"){}
};
struct BrakePlan { double t1,t2,t3,distance,peak; };
// Zero-initial-acceleration specialization of motion_utils
// calcDecelDistWithJerkAndAccConstraints: triangular/trapezoidal signed jerk,
// ending at target speed with zero acceleration. ROS-linked regression compares
// this distance with the existing utility rather than inventing new limits.
inline BrakePlan brake_plan(double speed,double target,double amin,double jmin,double jmax)
{
  if(!std::isfinite(speed+target+amin+jmin+jmax) || speed<0 || target<0 || target>speed || amin>=0 || jmin>=0 || jmax<=0)
    throw std::logic_error("invalid reachable-tail state");
  const double loss=speed-target,b=-amin;
  if(loss<1e-10)return {0,0,0,0,0};
  const double peak=std::min(b,std::sqrt(2*loss/(1/(-jmin)+1/jmax)));
  const double t1=peak/(-jmin),t3=peak/jmax;
  const double t2=std::max(0.,(loss-.5*peak*(t1+t3))/peak);
  double x=0,v=speed,a=0;
  auto advance=[&](double j,double dt){x+=v*dt+.5*a*dt*dt+j*dt*dt*dt/6;v+=a*dt+.5*j*dt*dt;a+=j*dt;};
  advance(jmin,t1);advance(0,t2);advance(jmax,t3);
  if(!std::isfinite(x) || x<0 || std::abs(v-target)>1e-6 || std::abs(a)>1e-6)
    throw std::logic_error("invalid reachable-tail brake plan");
  return {t1,t2,t3,x,peak};
}
inline Proposal make(const std::vector<double> & arc, const std::vector<double> & caps,
  double v, double a, double target_acc, double hold, double amin,
  double jmin, double jmax, const std::function<double(double)> & acceleration_limit,
  const bool retry_remote_tail=true)
{
  if(arc.size()<2 || arc.size()!=caps.size() || arc.front()!=0 ||
     !std::isfinite(v+a+target_acc+hold+amin+jmin+jmax) || v<0 || hold<=0 || hold>3.01 ||
     amin>=0 || jmin>=0 || jmax<=0 || target_acc<amin || !acceleration_limit)
    throw std::logic_error("invalid speed-grid input");
  for(size_t i=0;i<arc.size();++i)
    if(!std::isfinite(arc[i]+caps[i]) || caps[i]<0 || (i && arc[i]<=arc[i-1]))
      throw std::logic_error("invalid speed-grid geometry/caps");
  // Do not silently lower the actual initial speed to an already infeasible cap.
  if(v>caps.front()+1e-5)throw std::logic_error("initial upstream cap infeasible");
  Proposal out;out.samples.push_back({0,0,v,a});out.progress=0;
  double t=0,x=0;
  constexpr double dt=.01;
  bool covered=false;
  for(size_t n=0;v>1e-7 && !covered && n<6000;++n) {
    if(t>=hold && std::abs(a)<1e-12) {
      // Preserve the entire H-prefix and its jerk-limited release to a=0.
      // Find the most restrictive EXISTING lower cap and delay deceleration
      // until its reachable latest start. No new stop is introduced.
      const State checkpoint{t,x,v,a};
      const size_t prefix_samples=out.samples.size();
      constexpr std::array<double,5> factors{1.0,.875,.75,.625,.5};
      const size_t attempts=retry_remote_tail?factors.size():1;
      for(size_t attempt=0;attempt<attempts;++attempt) {
        t=checkpoint.t;x=checkpoint.s;v=checkpoint.v;a=checkpoint.a;
        covered=false;out.samples.resize(prefix_samples);
        out.tail_attempts=attempt+1;out.tail_factor=factors[attempt];
        size_t connected_caps=0;
        try {
          while(v>1e-7 && x<arc.back()-1e-7) {
        size_t selected=arc.size();double latest_start=arc.back();BrakePlan selected_plan{};
        for(size_t i=0;i<arc.size();++i) {
          if(arc[i]<=x+1e-7 || caps[i]>=v-1e-7)continue;
          const auto plan=brake_plan(v,caps[i],amin*factors[attempt],jmin,jmax);
          const double start=arc[i]-plan.distance;
          if(start<latest_start){latest_start=start;selected=i;selected_plan=plan;}
        }
        if(selected==arc.size()) {
          t+=(arc.back()-x)/v;x=arc.back();covered=true;out.samples.push_back({t,x,v,0});break;
        }
        if(latest_start<x-1e-6) {
          if(connected_caps>0)throw RemoteTailConnectionFailure{};
          throw std::logic_error("existing cap needs braking inside fixed prefix");
        }
        const auto protected_vertex=std::lower_bound(arc.begin(),arc.end(),x);
        if(protected_vertex!=arc.end() && latest_start<*protected_vertex-1e-7)
          throw std::logic_error("cap tail would alter protected prefix interpolation");
        if(latest_start>x+1e-9){t+=(latest_start-x)/v;x=latest_start;out.samples.push_back({t,x,v,0});}
        auto segment=[&](double jerk,double duration) {
          double left=duration;
          while(left>1e-10) {
            const double step=std::min(dt,left);
            x+=v*step+.5*a*step*step+jerk*step*step*step/6;
            v+=a*step+.5*jerk*step*step;a+=jerk*step;t+=step;left-=step;
            if(v< -1e-6)throw std::logic_error("tail reverses velocity");
            out.samples.push_back({t,x,std::max(0.,v),a});
          }
        };
        segment(jmin,selected_plan.t1);segment(0,selected_plan.t2);segment(jmax,selected_plan.t3);
        if(std::abs(x-arc[selected])>1e-5 || std::abs(v-caps[selected])>1e-6 || std::abs(a)>1e-6)
          throw std::logic_error("tail cap connection mismatch");
        x=arc[selected];v=caps[selected];a=0;
        out.samples.back()={t,x,v,a};
        ++connected_caps;
          }
          break; // A complete tail is retained; no need for weaker alternatives.
        } catch(const RemoteTailConnectionFailure &) {
          // Only a greedy REMOTE cap connection can trigger retry. Actual
          // initial state, H-prefix, near-stop and malformed-input failures
          // propagate unchanged. No safety checks or speed caps are relaxed.
          if(attempt+1==attempts)throw;
        }
      }
      if(x>=arc.back()-1e-7)covered=true;
      break;
    }
    const double cap=acceleration_limit(v);
    if(!std::isfinite(cap) || cap<=0)throw std::logic_error("missing acceleration envelope");
    const double step_duration=t<hold ? std::min(dt,hold-t) : dt;
    const double faster_cap=acceleration_limit(v+std::max({0.,a,cap})*step_duration);
    if(!std::isfinite(faster_cap) || faster_cap<=0)throw std::logic_error("missing future acceleration envelope");
    // Release to zero acceleration after the bounded observation window,
    // then connect only to existing caps/stops using the reachable tail above.
    const double desired=t<hold ? std::min({target_acc,cap,faster_cap}) : 0.;
    const double jerk=desired>a ? jmax : jmin;
    const double duration=std::min(step_duration,std::abs(desired-a)/std::abs(jerk));
    const double next_a=a+jerk*duration;
    // Exact integral: ramp to target, then hold target for the rest of dt.
    auto velocity_at=[&](double z){const double ramp=std::min(z,duration);
      return v+a*ramp+.5*jerk*ramp*ramp+next_a*(z-ramp);};
    auto distance_at=[&](double z){const double ramp=std::min(z,duration);
      const double vr=v+a*ramp+.5*jerk*ramp*ramp;
      const double rest=z-ramp;
      return v*ramp+.5*a*ramp*ramp+jerk*ramp*ramp*ramp/6+
        vr*rest+.5*next_a*rest*rest;};
    double used=step_duration;
    if(velocity_at(step_duration)<=0) {
      double lo=0,hi=step_duration;
      for(int k=0;k<40;++k){const double m=(lo+hi)/2;if(velocity_at(m)>0)lo=m;else hi=m;}
      used=hi;
    }
    if(x+distance_at(used)>=arc.back()) {
      double lo=0,hi=used;
      for(int k=0;k<40;++k){const double m=(lo+hi)/2;if(x+distance_at(m)<arc.back())lo=m;else hi=m;}
      used=hi;covered=true;
    }
    const double next_v=std::max(0.,velocity_at(used));
    x+=distance_at(used);t+=used;a=a+jerk*std::min(used,duration);v=next_v;
    if(!std::isfinite(x+v+a) || x<out.samples.back().s)
      throw std::logic_error("invalid integrated motion");
    out.samples.push_back({t,x,v,a});
    if(t<=hold+1e-8)out.progress=x;
    if(x>arc.back()+1e-6)throw std::logic_error("invalid geometry coverage");
  }
  if(v>1e-7 && !covered)throw std::logic_error("unbounded motion coverage");
  for(size_t i=0;i<arc.size();++i) {
    double planned=covered ? out.samples.back().v : 0.;
    if(arc[i]<out.samples.back().s) {
      const auto hi=std::upper_bound(out.samples.begin(),out.samples.end(),arc[i],
        [](double s,const State & p){return s<p.s;});
      const auto & b=*hi;const auto & a=*(hi-1);const double u=(arc[i]-a.s)/(b.s-a.s);
      planned=std::sqrt(std::max(0.,a.v*a.v+u*(b.v*b.v-a.v*a.v)));
    }
    if(planned>caps[i]+1e-5)throw std::logic_error("upstream cap/stop infeasible");
    out.speeds.push_back(std::min(planned,caps[i]));
  }
  return out;
}
// Additional candidate, never a replacement for make(). It is defined only
// for a uniform positive hard ceiling with sufficient path coverage. Its
// entire observation horizon is retained, but acceleration is released before
// the existing speed ceiling: v + a^2/(2 |jmin|) <= cap. Unsupported inputs
// throw, leaving selection/fallback and all downstream safety gates unchanged.
inline Proposal make_uniform_cap_coast(const std::vector<double>& arc,
  const std::vector<double>& caps,double v,double a,double target,double H,
  double jmin,double jmax,const std::function<double(double)>& limit,
  const std::function<bool()>& budget_available = {})
{
  if(arc.size()<2||arc.size()!=caps.size()||arc.front()!=0||
     !std::isfinite(v+a+target+H+jmin+jmax)||v<=0||a<0||target<=0||
     H<=0||H>3.01||jmin>=0||jmax<=0||!limit)throw std::logic_error("invalid uniform cap-coast input");
  const double cap=caps.front(), J=-jmin;
  for(size_t i=0;i<arc.size();++i)
    if(!std::isfinite(arc[i]+caps[i])||caps[i]<=0||caps[i]!=cap||
       (i&&arc[i]<=arc[i-1]))throw std::logic_error("cap-coast requires uniform positive hard caps");
  const double initial_limit=limit(v);
  if(!std::isfinite(initial_limit)||initial_limit<=0||a>initial_limit+1e-9)
    throw std::logic_error("initial acceleration outside cap-coast scope");
  if(v+a*a/(2*J)>cap+1e-9)throw std::logic_error("initial release infeasible");
  Proposal out;out.progress=0;out.samples.push_back({0,0,v,a});
  double t=0,s=0;
  for(size_t n=0;s<arc.back()-1e-9&&n<100000;++n){
    if(budget_available&&!budget_available())
      throw std::logic_error("cap-coast generation budget exhausted");
    double dt=t<H?std::min(.01,H-t):.01;
    // Resolve the release-to-zero event exactly, rather than smearing the
    // last negative-jerk fraction over a whole integration step.
    if(a>1e-10&&a<J*dt)dt=a/J;
    const double physical=std::min(limit(v),limit(v+std::max(a,limit(v))*dt));
    if(!std::isfinite(physical)||physical<=0)throw std::logic_error("invalid calibration");
    const double desired=t<H?std::min(target,physical):0.;
    double j=std::clamp((desired-a)/dt,jmin,jmax);
    const auto barrier=[&](double z){const double an=std::max(0.,a+z*dt);
      return v+a*dt+.5*z*dt*dt+an*an/(2*J);};
    const double lower=std::max(jmin,-a/dt);
    if(barrier(j)>cap){
      if(barrier(lower)>cap+1e-8)throw std::logic_error("release barrier infeasible");
      double lo=lower,hi=j;for(int k=0;k<60;++k){double m=(lo+hi)/2;
        if(barrier(m)<=cap)lo=m;else hi=m;}j=lo;
    }
    const auto dist=[&](double z){return v*z+.5*a*z*z+j*z*z*z/6;};
    if(s+dist(dt)>arc.back()) {double lo=0,hi=dt;for(int k=0;k<60;++k){double m=(lo+hi)/2;
      if(s+dist(m)<arc.back())lo=m;else hi=m;}dt=hi;}
    const double previous_a=a,previous_v=v;
    s+=dist(dt);v+=a*dt+.5*j*dt*dt;a=std::max(0.,a+j*dt);t+=dt;
    if(a<1e-10)a=0.;
    // Check actual acceleration against the calibration at its own speed,
    // including bounded interior samples. A sharp valid calibration can
    // outrun jerk-limited release: reject, never clamp the measured state.
    for(unsigned sample=0;sample<=4;++sample){
      const double z=dt*sample/4.;
      const double sample_v=previous_v+previous_a*z+.5*j*z*z;
      const double sample_a=std::max(0.,previous_a+j*z);
      const double sample_limit=limit(sample_v);
      if(!std::isfinite(sample_limit)||sample_limit<=0||sample_a>sample_limit+1e-9)
        throw std::logic_error("cap-coast acceleration envelope infeasible");
    }
    if(v>cap+1e-8||!std::isfinite(s+v+a))throw std::logic_error("hard cap violation");
    out.samples.push_back({t,s,v,a});if(t<=H+1e-8)out.progress=s;
  }
  if(s<arc.back()-1e-7||t<H)throw std::logic_error("insufficient cap-coast horizon coverage");
  for(double x:arc){auto hi=std::upper_bound(out.samples.begin(),out.samples.end(),x,
      [](double x,const auto&z){return x<z.s;});
    if(hi==out.samples.end()){out.speeds.push_back(out.samples.back().v);continue;}
    const auto&b=*hi;const auto&a=*(hi-1);const double u=(x-a.s)/(b.s-a.s);
    const double speed=std::sqrt(std::max(0.,a.v*a.v+u*(b.v*b.v-a.v*a.v)));
    if(speed>cap+1e-8)throw std::logic_error("interpolated cap violation");out.speeds.push_back(speed);
  }
  return out;
}
} // namespace bounded_speed_grid
