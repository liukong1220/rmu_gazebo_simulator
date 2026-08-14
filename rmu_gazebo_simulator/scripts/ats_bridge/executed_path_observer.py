#!/usr/bin/env python3
# Copyright 2026 ATS 2026 Sentry Project
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

"""Publish the path the robot actually travelled, for RViz and for scoring.

This is a read-only observer. It never touches ``/cmd_vel_mpc`` or
``/motion_control`` ownership, and it is not a copy of any planned path:

* the samples come from ``/localization`` (the formal localization output of
  Point-LIO -> loam_interface -> sensor_scan_generation), not from ground
  truth, not from ``/minco/reference_path`` and not from the MPC prediction;
* samples are only appended while navigation is active, so an idle robot does
  not accumulate a stationary blob;
* the buffer is cleared on emergency stop so a stale trail cannot be mistaken
  for live tracking after a fault.

Published topic: ``/ats_swerve_mpc/executed_path`` (``nav_msgs/Path``) in the
``/localization`` header frame (``odom`` in the ATS contract).
"""

from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool


class ExecutedPathObserver(Node):
    """Accumulate the realised trajectory from ``/localization``."""

    def __init__(self) -> None:
        super().__init__("executed_path_observer")

        self.declare_parameter("odom_topic", "/localization")
        self.declare_parameter("output_topic", "/ats_swerve_mpc/executed_path")
        self.declare_parameter("emergency_stop_topic", "/planner/emergency_stop")
        self.declare_parameter("reference_path_topic", "/minco/reference_path")
        # Only append while a reference is fresh: that is the definition of
        # "active navigation" for this profile.
        self.declare_parameter("reference_timeout", 2.0)
        self.declare_parameter("require_active_navigation", True)
        self.declare_parameter("min_sample_distance", 0.02)
        self.declare_parameter("max_poses", 5000)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("frame_id", "")

        self.min_sample_distance = float(
            self.get_parameter("min_sample_distance").value
        )
        self.max_poses = int(self.get_parameter("max_poses").value)
        self.reference_timeout = float(self.get_parameter("reference_timeout").value)
        self.require_active = bool(
            self.get_parameter("require_active_navigation").value
        )
        self.frame_override = str(self.get_parameter("frame_id").value)

        self.poses: list[PoseStamped] = []
        self.frame_id = "odom"
        self.last_reference_time: float | None = None
        self.emergency_stop = False

        reliable = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # localization_fusion publishes /localization as BEST_EFFORT. A RELIABLE
        # subscription is an incompatible-QoS request, so DDS matches nothing and
        # the observer silently receives no pose at all.
        sensor_stream = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter("output_topic").value), reliable
        )

        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            sensor_stream,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            self._emergency_callback,
            reliable,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("reference_path_topic").value),
            self._reference_callback,
            reliable,
        )

        rate = max(0.5, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            "executed_path_observer: '%s' -> '%s'"
            % (
                self.get_parameter("odom_topic").value,
                self.get_parameter("output_topic").value,
            )
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _emergency_callback(self, msg: Bool) -> None:
        latched = bool(msg.data)
        if latched and not self.emergency_stop:
            # A stale trail after a fault would read as "still tracking".
            self.poses.clear()
        self.emergency_stop = latched

    def _reference_callback(self, msg: Path) -> None:
        if msg.poses:
            self.last_reference_time = self._now()

    def _navigation_active(self) -> bool:
        if not self.require_active:
            return True
        if self.emergency_stop:
            return False
        if self.last_reference_time is None:
            return False
        if self.reference_timeout <= 0.0:
            return True
        return (self._now() - self.last_reference_time) <= self.reference_timeout

    def _odom_callback(self, msg: Odometry) -> None:
        if not self._navigation_active():
            return

        self.frame_id = self.frame_override or msg.header.frame_id or "odom"

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = self.frame_id
        pose.pose = msg.pose.pose

        if self.poses:
            previous = self.poses[-1].pose.position
            moved = math.hypot(
                pose.pose.position.x - previous.x, pose.pose.position.y - previous.y
            )
            if moved < self.min_sample_distance:
                return

        self.poses.append(pose)
        if self.max_poses > 0 and len(self.poses) > self.max_poses:
            del self.poses[: len(self.poses) - self.max_poses]

    def _publish(self) -> None:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        path.poses = list(self.poses)
        self.path_pub.publish(path)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ExecutedPathObserver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
