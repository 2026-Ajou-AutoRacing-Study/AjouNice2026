#!/usr/bin/env python3
"""Convert MORAI traffic-light status packets into Autoware signal groups."""

import os

import rospy
import yaml
from autoware_perception_msgs.msg import (
    TrafficLightElement,
    TrafficLightGroup,
    TrafficLightGroupArray,
)
from morai_msgs.msg import GetTrafficLightStatus


# MORAI GetTrafficLightStatus uses a bit field for simultaneous lamps.
MORAI_RED = 1
MORAI_YELLOW = 4
MORAI_GREEN = 16
MORAI_GREEN_LEFT = 32


def traffic_light_element(color, shape=TrafficLightElement.CIRCLE):
    """Build one reliable, currently illuminated Autoware signal element."""
    output = TrafficLightElement()
    output.color = int(color)
    output.shape = int(shape)
    output.status = TrafficLightElement.SOLID_ON
    output.confidence = 1.0
    return output


def elements_from_morai_state(traffic_light_status):
    """Convert MORAI's red/yellow/green/left-arrow bit field."""
    status = int(traffic_light_status)
    elements = []
    # MORAI uses -1 for "no traffic-light information".  It is not a bit
    # mask with every lamp enabled.
    if status < 0:
        return [traffic_light_element(TrafficLightElement.UNKNOWN)]
    if status & MORAI_RED:
        elements.append(traffic_light_element(TrafficLightElement.RED))
    if status & MORAI_YELLOW:
        elements.append(traffic_light_element(TrafficLightElement.AMBER))
    if status & MORAI_GREEN:
        elements.append(traffic_light_element(TrafficLightElement.GREEN))
    if status & MORAI_GREEN_LEFT:
        elements.append(
            traffic_light_element(
                TrafficLightElement.GREEN, TrafficLightElement.LEFT_ARROW
            )
        )
    if not elements:
        # MORAI -1 means that no lamp state is available.  Unknown must not be
        # treated as green by BVP, so it results in a conservative stop.
        elements.append(traffic_light_element(TrafficLightElement.UNKNOWN))
    return elements


def load_traffic_light_map(mapping_file):
    """Load {MORAI signal string: Lanelet regulatory group ID} mapping."""
    if not mapping_file:
        return {}
    with open(mapping_file, "r") as stream:
        data = yaml.safe_load(stream) or {}
    entries = data.get("traffic_light_groups", data)
    if not isinstance(entries, dict):
        raise ValueError("traffic_light_groups must be a mapping")
    return {str(key).strip(): int(value) for key, value in entries.items()}


def make_traffic_light_group(message, group_by_morai_id):
    """Translate one MORAI status message, or return None when unmapped."""
    signal_id = str(message.trafficLightIndex).strip()
    group_id = group_by_morai_id.get(signal_id)
    if group_id is None:
        return None
    output = TrafficLightGroup()
    output.traffic_light_group_id = int(group_id)
    output.elements = elements_from_morai_state(message.trafficLightStatus)
    return output


def make_traffic_light_group_array(message, group_by_morai_id, now=None):
    """Build the BVP input message with a fresh state timestamp."""
    output = TrafficLightGroupArray()
    output.stamp = message.header.stamp
    if output.stamp == rospy.Time():
        output.stamp = now if now is not None else rospy.Time.now()
    group = make_traffic_light_group(message, group_by_morai_id)
    if group is not None:
        output.traffic_light_groups = [group]
    return output


class MoraiTrafficLightBridge:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/GetTrafficLightStatus")
        output_topic = rospy.get_param(
            "~output_topic",
            "/perception/traffic_light_recognition/traffic_signals",
        )
        default_mapping_file = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            "config",
            "morai_traffic_light_map_v60.yaml",
        )
        mapping_file = rospy.get_param("~mapping_file", default_mapping_file)
        self.group_by_morai_id = load_traffic_light_map(mapping_file)
        if not self.group_by_morai_id:
            raise RuntimeError("MORAI traffic-light mapping is empty: {}".format(mapping_file))
        self.publisher = rospy.Publisher(
            output_topic, TrafficLightGroupArray, queue_size=10
        )
        self.subscriber = rospy.Subscriber(
            input_topic, GetTrafficLightStatus, self.on_traffic_light, queue_size=10
        )
        rospy.loginfo(
            "MORAI traffic-light bridge: %s -> %s (%d signal mappings)",
            input_topic,
            output_topic,
            len(self.group_by_morai_id),
        )

    def on_traffic_light(self, message):
        output = make_traffic_light_group_array(message, self.group_by_morai_id)
        if not output.traffic_light_groups:
            rospy.logwarn_throttle(
                2.0,
                "MORAI traffic-light ID %r is not mapped to the active Lanelet map",
                message.trafficLightIndex,
            )
        self.publisher.publish(output)


def main():
    rospy.init_node("morai_traffic_light_bridge")
    MoraiTrafficLightBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
