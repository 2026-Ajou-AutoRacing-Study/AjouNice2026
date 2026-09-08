// Copyright 2024 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__MISSION_PLANNER_UNIVERSE__SERVICE_UTILS_HPP_
#define AUTOWARE__MISSION_PLANNER_UNIVERSE__SERVICE_UTILS_HPP_

#include <autoware_utils/system/stop_watch.hpp>

#include <autoware_common_msgs/ResponseStatus.h>
#include <ros/ros.h>

#include <functional>
#include <stdexcept>
#include <string>

namespace service_utils
{

using ResponseStatus = autoware_common_msgs::ResponseStatus;
using ResponseStatusCode = ResponseStatus::_code_type;

class ServiceException : public std::exception
{
public:
  ServiceException(ResponseStatusCode code, const std::string & message, bool success = false)
  {
    success_ = success;
    code_ = code;
    message_ = message;
  }

  template <class StatusT>
  void set(StatusT & status) const
  {
    status.success = success_;
    status.code = code_;
    status.message = message_;
  }

  const char * what() const noexcept override { return message_.c_str(); }

private:
  bool success_;
  ResponseStatusCode code_;
  std::string message_;
};

ServiceException ServiceUnready(const std::string & message);
ServiceException TransformError(const std::string & message);

template <class T, class Req, class Res>
std::function<bool(Req &, Res &)> handle_exception(
  void (T::*callback)(const Req &, Res &), T * instance)
{
  return [instance, callback](Req & req, Res & res) {
    autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
    try {
      (instance->*callback)(req, res);
    } catch (const ServiceException & error) {
      error.set(res->status);
    }
    instance->publish_processing_time(stop_watch);
    return true;
  };
}

template <class ServiceT>
ResponseStatus sync_call(ros::ServiceClient & client, ServiceT & service)
{
  if (!client.exists()) {
    throw ServiceUnready(client.getService());
  }
  if (!client.call(service)) {
    throw ServiceUnready(client.getService());
  }
  return service.response.status;
}

}  // namespace service_utils

#endif  // AUTOWARE__MISSION_PLANNER_UNIVERSE__SERVICE_UTILS_HPP_
