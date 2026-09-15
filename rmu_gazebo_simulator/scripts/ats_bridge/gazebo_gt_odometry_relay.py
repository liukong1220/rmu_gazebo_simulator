#!/usr/bin/env python3
"""Relay Gazebo chassis ground-truth odometry into the ATS /odometry contract.

Gazebo SwerveDrive4WS publishes world-frame chassis pose on
/<robot>/odometry (bridged as /<robot>/chassis_odometry_gt). Point-LIO
odometry diverges under red_box motion; MuJoCo already feeds sim truth.
This relay converts world pose into the odom frame expected by
localization_fusion:

  odom = R(-spawn_yaw) * (world - spawn_xy)

so the robot starts near (0,0) in odom. Fusion still applies the configured
initial map->odom so /localization matches the static-map / red_box frame
(spawn map pose ~ initial_map_to_odom).
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


def _yaw_from_quat(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class GazeboGtOdometryRelay(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_gt_odometry_relay")
        self.declare_parameter("gt_odom_topic", "/red_standard_robot1/chassis_odometry_gt")
        self.declare_parameter("output_odom_topic", "/odometry")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "gimbal_yaw_odom")
        # Gazebo world spawn of red_standard_robot1 on rmuc_2025 (gz_world.yaml).
        self.declare_parameter("spawn_world_x", 4.75)
        self.declare_parameter("spawn_world_y", 9.00)
        self.declare_parameter("spawn_world_yaw", 0.0)
        self.declare_parameter("publish_tf", True)

        self.gt_topic = str(self.get_parameter("gt_odom_topic").value)
        self.out_topic = str(self.get_parameter("output_odom_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.spawn_x = float(self.get_parameter("spawn_world_x").value)
        self.spawn_y = float(self.get_parameter("spawn_world_y").value)
        self.spawn_yaw = float(self.get_parameter("spawn_world_yaw").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(Odometry, self.out_topic, sensor_qos)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Odometry, self.gt_topic, self._on_gt, sensor_qos)
        self.get_logger().info(
            "Relaying GT '%s' -> '%s' frames %s->%s spawn_world=(%.3f,%.3f,%.3f)"
            % (
                self.gt_topic,
                self.out_topic,
                self.odom_frame,
                self.base_frame,
                self.spawn_x,
                self.spawn_y,
                self.spawn_yaw,
            )
        )

    def _on_gt(self, msg: Odometry) -> None:
        wx = msg.pose.pose.position.x
        wy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        wyaw = _yaw_from_quat(q.z, q.w)

        dx = wx - self.spawn_x
        dy = wy - self.spawn_y
        c = math.cos(-self.spawn_yaw)
        s = math.sin(-self.spawn_yaw)
        ox = c * dx - s * dy
        oy = s * dx + c * dy
        oyaw = wyaw - self.spawn_yaw
        zq = _quat_from_yaw(oyaw)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = ox
        out.pose.pose.position.y = oy
        out.pose.pose.position.z = msg.pose.pose.position.z
        out.pose.pose.orientation.x = zq[0]
        out.pose.pose.orientation.y = zq[1]
        out.pose.pose.orientation.z = zq[2]
        out.pose.pose.orientation.w = zq[3]
        out.twist = msg.twist
        self.pub.publish(out)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = out.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = ox
            tf.transform.translation.y = oy
            tf.transform.translation.z = out.pose.pose.position.z
            tf.transform.rotation = out.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)


def main() -> None:
    rclpy.init()
    node = GazeboGtOdometryRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
