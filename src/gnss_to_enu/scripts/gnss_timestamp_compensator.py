#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import copy

import rospy
from sensor_msgs.msg import NavSatFix


def compensated_stamp_seconds(original_stamp_seconds: float, offset_seconds: float) -> float:
    adjusted = float(original_stamp_seconds) - float(offset_seconds)
    if adjusted < 0.0:
        raise ValueError(
            "Adjusted GNSS stamp is negative: {:.9f} - {:.9f} = {:.9f}".format(
                float(original_stamp_seconds),
                float(offset_seconds),
                adjusted,
            )
        )
    return adjusted


class GNSSTimestampCompensator:
    def __init__(self):
        rospy.init_node("gnss_timestamp_compensator", anonymous=False)

        self.sub_topic = rospy.get_param("~sub_topic", "/handsfree/rtk/gnss")
        self.pub_topic = rospy.get_param("~pub_topic", "/handsfree/rtk/gnss_time_aligned")
        self.offset_seconds = float(rospy.get_param("~offset_seconds", 0.0))
        self.frame_id = rospy.get_param("~frame_id", "")

        self.pub = rospy.Publisher(self.pub_topic, NavSatFix, queue_size=100)
        self.sub = rospy.Subscriber(self.sub_topic, NavSatFix, self.callback, queue_size=100)

        rospy.loginfo(
            "[GNSS timestamp compensator] %s -> %s, adjusted_stamp = original_stamp - %.9f s",
            self.sub_topic,
            self.pub_topic,
            self.offset_seconds,
        )

    def callback(self, msg: NavSatFix) -> None:
        out = copy.deepcopy(msg)
        try:
            adjusted = compensated_stamp_seconds(msg.header.stamp.to_sec(), self.offset_seconds)
        except ValueError as exc:
            rospy.logerr_throttle(1.0, "[GNSS timestamp compensator] %s", exc)
            return

        out.header.stamp = rospy.Time.from_sec(adjusted)
        if self.frame_id:
            out.header.frame_id = self.frame_id

        rospy.loginfo_throttle(
            5.0,
            "[GNSS timestamp compensator] original=%.6f adjusted=%.6f offset=%.6f",
            msg.header.stamp.to_sec(),
            adjusted,
            self.offset_seconds,
        )
        self.pub.publish(out)


def main() -> None:
    GNSSTimestampCompensator()
    rospy.spin()


if __name__ == "__main__":
    main()
