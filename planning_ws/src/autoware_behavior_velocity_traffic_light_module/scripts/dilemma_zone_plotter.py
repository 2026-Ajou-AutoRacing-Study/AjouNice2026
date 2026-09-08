#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import os

import rospkg
import matplotlib.pyplot as plt
import numpy as np
import yaml


def get_params_from_yaml(traffic_light_config=None, common_config=None):
    packages = rospkg.RosPack()
    traffic_light_config = traffic_light_config or os.path.join(
        packages.get_path("autoware_behavior_velocity_traffic_light_module"),
        "config/traffic_light.param.yaml")
    common_config = common_config or os.path.join(
        packages.get_path("autoware_behavior_velocity_planner_common"),
        "config/behavior_velocity_planner_common.param.yaml")

    def load(path):
        with open(path, "r") as stream:
            params = yaml.safe_load(stream)
        return params["/**"]["ros__parameters"] if "/**" in params else params

    traffic = load(traffic_light_config)
    common = load(common_config)
    return traffic["traffic_light"]["yellow_lamp_period"], common["max_accel"], common["delay_response_time"]


def plot(max_ego_vel, params):
    yellow_lamp_period, max_accel, delay_response_time = params

    # create data to plot
    ego_vel = np.linspace(0, round(max_ego_vel), 100)
    moving_distance_during_yellow_light = yellow_lamp_period * ego_vel
    stop_distance = delay_response_time * ego_vel - ego_vel**2 / (2 * max_accel)

    # plot
    fig, ax = plt.subplots()
    ax.plot(moving_distance_during_yellow_light, ego_vel, color="blue")
    ax.plot(stop_distance, ego_vel, color="red")

    # label for the plot
    ax.set_xlabel("moving distance [m]")
    ax.set_ylabel("ego velocity [m/s]")

    # colorize the regions
    min_distance = np.minimum(moving_distance_during_yellow_light, stop_distance)
    max_distance = np.maximum(moving_distance_during_yellow_light, stop_distance)
    ax.fill_betweenx(
        ego_vel,
        stop_distance,
        moving_distance_during_yellow_light,
        where=(moving_distance_during_yellow_light >= stop_distance),
        facecolor="lightgreen",
        interpolate=True,
        alpha=0.3,
    )
    ax.fill_betweenx(
        ego_vel,
        stop_distance,
        moving_distance_during_yellow_light,
        where=(moving_distance_during_yellow_light < stop_distance),
        facecolor="orange",
        interpolate=True,
        alpha=0.3,
    )
    ax.fill_betweenx(ego_vel, min_distance, facecolor="yellow", interpolate=True, alpha=0.1)
    ax.fill_between(max_distance, ego_vel, facecolor="red", interpolate=True, alpha=0.05)

    # init dot lines for the cursor
    hline = ax.axhline(y=0, color="gray", linestyle="--")
    vline = ax.axvline(x=0, color="gray", linestyle="--")

    # capture the cursor
    def on_mouse_move(event):
        if event.inaxes:
            hline.set_ydata(event.ydata)
            vline.set_xdata(event.xdata)
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect("motion_notify_event", on_mouse_move)

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--vel", default=16.7, type=float, help="maximum ego velocity [m/s]")
    parser.add_argument("--traffic-light-config")
    parser.add_argument("--common-config")
    args = parser.parse_args()

    params = get_params_from_yaml(args.traffic_light_config, args.common_config)

    max_ego_vel = args.vel
    plot(max_ego_vel, params)
