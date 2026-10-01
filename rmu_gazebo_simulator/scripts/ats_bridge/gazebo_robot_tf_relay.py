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

"""Expose the namespaced robot_state_publisher tree on the global /tf.

spawn_robots.launch.py runs robot_state_publisher with /tf remapped to
/<robot>/tf, so RViz and every navigation node on the global tree never see
base_footprint, the chassis or the swerve modules. The global tree is rooted
at odom -> gimbal_yaw_odom (GT relay or sensor_scan_generation), while the
URDF is rooted at base_footprint -> chassis -> gimbal_yaw_odom. Forwarding the
URDF edges verbatim would give gimbal_yaw_odom two parents, so this relay:

  * drops the chassis -> gimbal_yaw_odom edge and publishes its inverse chain
    gimbal_yaw_odom -> base_footprint (base_footprint stays on the ground and
    follows the revolute big-yaw joint);
  * drops edges whose child already has a global owner (front_mid360 comes
    from the launch static TF);
  * forwards every other edge unchanged.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate(q, v):
    x, y, z, _ = _quat_mul(
        _quat_mul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3])
    )
    return (x, y, z)


def _to_tuple(transform):
    t = transform.translation
    r = transform.rotation
    return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)


def _compose(a, b):
    """Return a * b for (translation, quaternion) pairs."""
    ta, qa = a
    tb, qb = b
    rb = _rotate(qa, tb)
    return (ta[0] + rb[0], ta[1] + rb[1], ta[2] + rb[2]), _quat_mul(qa, qb)


def _inverse(a):
    t, q = a
    q_inv = (-q[0], -q[1], -q[2], q[3])
    rt = _rotate(q_inv, t)
    return (-rt[0], -rt[1], -rt[2]), q_inv


class GazeboRobotTfRelay(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_robot_tf_relay")
        self.declare_parameter("input_tf_topic", "/red_standard_robot1/tf")
        self.declare_parameter(
            "input_tf_static_topic", "/red_standard_robot1/tf_static"
        )
        self.declare_parameter("robot_base_frame", "gimbal_yaw_odom")
        self.declare_parameter("base_footprint_frame", "base_footprint")
        self.declare_parameter("excluded_child_frames", ["front_mid360"])

        self.robot_base_frame = str(self.get_parameter("robot_base_frame").value)
        self.footprint_frame = str(self.get_parameter("base_footprint_frame").value)
        self.excluded = set(self.get_parameter("excluded_child_frames").value)
        self.static_edges: dict[str, TransformStamped] = {}
        # Every static URDF edge published so far; re-sent as one latched
        # message because /tf_static subscribers only keep the latest sample.
        self.static_out: dict[str, TransformStamped] = {}

        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        dynamic_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.tf_pub = self.create_publisher(TFMessage, "/tf", dynamic_qos)
        self.tf_static_pub = self.create_publisher(TFMessage, "/tf_static", static_qos)
        self.create_subscription(
            TFMessage,
            str(self.get_parameter("input_tf_static_topic").value),
            self._on_static,
            static_qos,
        )
        self.create_subscription(
            TFMessage,
            str(self.get_parameter("input_tf_topic").value),
            self._on_dynamic,
            dynamic_qos,
        )

    def _on_static(self, msg: TFMessage) -> None:
        changed = False
        for tf in msg.transforms:
            self.static_edges[tf.child_frame_id] = tf
            if (
                tf.child_frame_id in self.excluded
                or tf.child_frame_id == self.robot_base_frame
            ):
                continue
            self.static_out[tf.child_frame_id] = tf
            changed = True
        if changed:
            self.tf_static_pub.publish(
                TFMessage(transforms=list(self.static_out.values()))
            )

    def _on_dynamic(self, msg: TFMessage) -> None:
        out = []
        for tf in msg.transforms:
            if tf.child_frame_id in self.excluded:
                continue
            if tf.child_frame_id == self.robot_base_frame:
                inverse = self._footprint_under_base(tf)
                if inverse is not None:
                    out.append(inverse)
                continue
            out.append(tf)
        if out:
            self.tf_pub.publish(TFMessage(transforms=out))

    def _footprint_under_base(self, parent_to_base: TransformStamped):
        # Walk static edges from the big-yaw parent (chassis) up to base_footprint.
        chain = _to_tuple(parent_to_base.transform)
        frame = parent_to_base.header.frame_id
        while frame != self.footprint_frame:
            edge = self.static_edges.get(frame)
            if edge is None:
                self.get_logger().warn(
                    "【Gazebo TF等待】等待到 %s 的静态边，再发布 %s -> %s"
                    % (frame, self.robot_base_frame, self.footprint_frame),
                    throttle_duration_sec=5.0,
                )
                return None
            chain = _compose(_to_tuple(edge.transform), chain)
            frame = edge.header.frame_id
        translation, rotation = _inverse(chain)
        out = TransformStamped()
        out.header.stamp = parent_to_base.header.stamp
        out.header.frame_id = self.robot_base_frame
        out.child_frame_id = self.footprint_frame
        out.transform.translation.x = translation[0]
        out.transform.translation.y = translation[1]
        out.transform.translation.z = translation[2]
        out.transform.rotation.x = rotation[0]
        out.transform.rotation.y = rotation[1]
        out.transform.rotation.z = rotation[2]
        out.transform.rotation.w = rotation[3]
        return out


def main() -> None:
    rclpy.init()
    node = GazeboRobotTfRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
