#!/usr/bin/env python3
"""Normalize native Gazebo gimbal ground truth into the ATS /odometry contract.

The drivetrain plugin publishes the actual gimbal_yaw_odom link world pose
and child-frame twist at the simulation stamp on /<robot>/gimbal_odometry_gt.
This relay only changes the world origin, never substitutes chassis pose for
the moving gimbal link. localization_fusion receives:

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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster




class GazeboGtOdometryRelay(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_gt_odometry_relay")
        self.declare_parameter("gt_odom_topic", "/red_standard_robot1/gimbal_odometry_gt")
        self.declare_parameter("gt_child_frame", "red_standard_robot1/gimbal_yaw_odom")
        self.declare_parameter("output_odom_topic", "/odometry")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "gimbal_yaw_odom")
        # Gazebo world spawn of red_standard_robot1 on rmuc_2025 (gz_world.yaml).
        self.declare_parameter("spawn_world_x", 4.75)
        self.declare_parameter("spawn_world_y", 9.00)
        self.declare_parameter("spawn_world_yaw", 0.0)
        self.declare_parameter("publish_tf", True)

        self.gt_topic = str(self.get_parameter("gt_odom_topic").value)
        self.gt_child_frame = str(self.get_parameter("gt_child_frame").value)
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
            "【Gazebo真值里程计】转发 GT '%s' -> '%s' 坐标系 %s->%s 出生点=(%.3f,%.3f,%.3f)"
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
        if msg.header.frame_id != "world" or msg.child_frame_id != self.gt_child_frame:
            self.get_logger().error(
                "【Gazebo真值里程计拒绝】收到坐标系 %s->%s，期望 world->%s（禁止回退到底盘）"
                % (msg.header.frame_id, msg.child_frame_id, self.gt_child_frame),
                throttle_duration_sec=5.0,
            )
            return
        wx = msg.pose.pose.position.x
        wy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        dx = wx - self.spawn_x
        dy = wy - self.spawn_y
        c = math.cos(-self.spawn_yaw)
        s = math.sin(-self.spawn_yaw)
        ox = c * dx - s * dy
        oy = s * dx + c * dy
        # Left-multiply the complete native quaternion by Rz(-spawn_yaw).
        # Roll/pitch and the gimbal joint's physical yaw remain observable.
        sz = math.sin(-0.5 * self.spawn_yaw)
        cw = math.cos(-0.5 * self.spawn_yaw)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = ox
        out.pose.pose.position.y = oy
        out.pose.pose.position.z = msg.pose.pose.position.z
        out.pose.pose.orientation.x = cw * q.x - sz * q.y
        out.pose.pose.orientation.y = cw * q.y + sz * q.x
        out.pose.pose.orientation.z = cw * q.z + sz * q.w
        out.pose.pose.orientation.w = cw * q.w - sz * q.z
        # The physical child is unchanged; native twist is already child-framed.
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
