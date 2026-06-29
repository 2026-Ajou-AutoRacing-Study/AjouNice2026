#!/usr/bin/env python3

import math
import rospy
from morai_msgs.msg import EgoVehicleStatus


class AngularUnitChecker:
    def __init__(self):
        self.prev_msg = None
        self.samples = []

        self.sub = rospy.Subscriber(
            "/Ego_topic",
            EgoVehicleStatus,
            self.callback,
            queue_size=10
        )

        rospy.loginfo("Checking MORAI Ego angular_velocity unit...")
        rospy.loginfo("Drive in a smooth turn for 10~20 seconds.")

    def normalize_deg_diff(self, diff):
        # heading이 359 -> 0 또는 0 -> 359를 지나갈 때 튀지 않게 보정.
        while diff > 180.0:
            diff -= 360.0
        while diff < -180.0:
            diff += 360.0
        return diff

    def callback(self, msg):
        if self.prev_msg is None:
            self.prev_msg = msg
            return

        t0 = self.prev_msg.header.stamp.to_sec()
        t1 = msg.header.stamp.to_sec()
        dt = t1 - t0

        if dt <= 0.0:
            self.prev_msg = msg
            return

        h0 = self.prev_msg.heading
        h1 = msg.heading

        dh_deg = self.normalize_deg_diff(h1 - h0)

        # heading 변화로부터 계산한 yaw rate.
        heading_rate_deg_s = dh_deg / dt
        heading_rate_rad_s = math.radians(heading_rate_deg_s)

        # Older local bags exposed angular_velocity; competition EgoVehicleStatus does not.
        angular_velocity = getattr(msg, "angular_velocity", None)
        wz = angular_velocity.z if angular_velocity is not None else None

        self.samples.append((heading_rate_deg_s, heading_rate_rad_s, wz))

        if len(self.samples) >= 20:
            recent = self.samples[-20:]

            avg_deg_s = sum(abs(x[0]) for x in recent) / len(recent)
            avg_rad_s = sum(abs(x[1]) for x in recent) / len(recent)
            wz_samples = [abs(x[2]) for x in recent if x[2] is not None]

            if not wz_samples:
                rospy.loginfo(
                    "avg heading_rate: %.3f deg/s = %.3f rad/s | no angular_velocity field",
                    avg_deg_s,
                    avg_rad_s
                )
                self.prev_msg = msg
                return

            avg_wz = sum(wz_samples) / len(wz_samples)

            rospy.loginfo(
                "avg heading_rate: %.3f deg/s = %.3f rad/s | avg angular_velocity.z: %.3f",
                avg_deg_s,
                avg_rad_s,
                avg_wz
            )

            if avg_wz < 1e-6:
                rospy.logwarn("angular_velocity.z is almost zero. Turn more strongly.")
            else:
                ratio_to_rad = avg_wz / avg_rad_s if avg_rad_s > 1e-6 else float("inf")
                ratio_to_deg = avg_wz / avg_deg_s if avg_deg_s > 1e-6 else float("inf")

                rospy.loginfo(
                    "ratio: wz/rad_s=%.2f, wz/deg_s=%.2f",
                    ratio_to_rad,
                    ratio_to_deg
                )

                if abs(ratio_to_rad - 1.0) < abs(ratio_to_deg - 1.0):
                    rospy.loginfo("LIKELY: angular_velocity is rad/s. Do NOT apply math.radians().")
                else:
                    rospy.loginfo("LIKELY: angular_velocity is deg/s. Apply math.radians().")

        self.prev_msg = msg


def main():
    rospy.init_node("check_morai_angular_unit")
    AngularUnitChecker()
    rospy.spin()


if __name__ == "__main__":
    main()
