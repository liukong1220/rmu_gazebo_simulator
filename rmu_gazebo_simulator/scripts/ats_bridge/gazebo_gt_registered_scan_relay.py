#!/usr/bin/env python3
"""Relay body-frame LiDAR into /registered_scan using GT-backed TF (Gazebo-only).

MuJoCo publishes /registered_scan already expressed in odom from sim truth.
Gazebo GT mode already owns /odometry + odom->gimbal_yaw_odom, but Point-LIO
/loam_interface still emit /registered_scan in a diverging LIO frame labeled as
odom. ROGMap then fuses GT pose with LIO clouds and corrupts the planning grid.

This node mirrors the MuJoCo / navi_minco_bit contract:
  /<robot>/livox/lidar (PointCloud2, sensor frame)
      --TF(odom chain)--> /registered_scan (odom)

Do NOT subscribe /livox/lidar: that topic is gz_livox_bridge CustomMsg for
Point-LIO. The PointCloud2 source is ros_gz on /<robot>/livox/lidar (RELIABLE).

Transform path: manual XYZ only (numpy). Gazebo/ros_gz PointCloud2 fields often
trip tf2_sensor_msgs.do_transform_cloud (PointFields/dtype assert). navi_minco_bit
and local loam/sensor_scan paths use PCL field-aware transforms; for Python we
rebuild a minimal xyz cloud in the target frame.

Ownership / QoS (aligned with loam_interface + navi_minco_bit cloud_registered):
  - Subscribe /<robot>/livox/lidar RELIABLE (ros_gz default).
  - Publish /registered_scan RELIABLE keep_last(5).

Only run when use_gazebo_gt_odometry:=true and Point-LIO/loam_interface are off.
"""

from __future__ import annotations

import struct

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Header


def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Unit quaternion -> 3x3 rotation matrix (ROS/tf2 convention)."""
    q = np.array([x, y, z, w], dtype=np.float64)
    if not np.isfinite(q).all():
        raise ValueError("non-finite quaternion in GT registered_scan TF")
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        raise ValueError("near-zero quaternion in GT registered_scan TF")
    x, y, z, w = (q / n).tolist()
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


_XYZ_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]


def _empty_xyz_cloud(stamp, frame_id: str) -> PointCloud2:
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return point_cloud2.create_cloud(header, _XYZ_FIELDS, [])


def _transform_xyz_cloud(
    msg: PointCloud2, tf_msg, min_range_m: float = 0.0
) -> PointCloud2:
    """Apply TransformStamped to xyz only; emit a dense xyz PointCloud2.

    Avoids tf2_sensor_msgs.do_transform_cloud, which asserts on Gazebo field
    layouts (extra/intensity type mismatches vs structured dtype).

    Gazebo/ros_gz Livox clouds often contain Inf for invalid returns; skip_nans
    alone does not drop them and will NaN-poison ROGMap occupancy.

    min_range_m drops body-frame near returns (robot self-hits) before TF so
    ego footprint cells are not stamped occupied at spawn.
    """
    pts = point_cloud2.read_points(
        msg, field_names=("x", "y", "z"), skip_nans=True
    )
    arr = np.fromiter(
        ((p[0], p[1], p[2]) for p in pts),
        dtype=np.dtype([("x", np.float32), ("y", np.float32), ("z", np.float32)]),
    )
    frame_id = tf_msg.header.frame_id
    if arr.size == 0:
        return _empty_xyz_cloud(msg.header.stamp, frame_id)

    xyz = np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float64, copy=False)
    # skip_nans does not remove +/- Inf invalid returns from Gazebo LiDAR.
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if min_range_m > 0.0 and xyz.size:
        r2 = xyz[:, 0] * xyz[:, 0] + xyz[:, 1] * xyz[:, 1]
        xyz = xyz[r2 >= (min_range_m * min_range_m)]
    if xyz.size == 0:
        return _empty_xyz_cloud(msg.header.stamp, frame_id)

    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    if not np.isfinite([t.x, t.y, t.z, q.x, q.y, q.z, q.w]).all():
        raise ValueError("non-finite TF translation/rotation for registered_scan")
    R = _quat_to_rot(q.x, q.y, q.z, q.w)
    out_xyz = (R @ xyz.T).T
    out_xyz[:, 0] += t.x
    out_xyz[:, 1] += t.y
    out_xyz[:, 2] += t.z
    out_xyz = out_xyz[np.isfinite(out_xyz).all(axis=1)]
    if out_xyz.size == 0:
        return _empty_xyz_cloud(msg.header.stamp, frame_id)

    header = Header()
    header.stamp = msg.header.stamp
    header.frame_id = frame_id
    # Explicit tuples avoid ambiguous ndarray iteration in create_cloud.
    points = [(float(px), float(py), float(pz)) for px, py, pz in out_xyz.astype(np.float32)]
    return point_cloud2.create_cloud(header, _XYZ_FIELDS, points)


class GazeboGtRegisteredScanRelay(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_gt_registered_scan_relay")
        self.declare_parameter(
            "input_cloud_topic", "/red_standard_robot1/livox/lidar"
        )
        self.declare_parameter("output_cloud_topic", "/registered_scan")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("tf_timeout_sec", 0.10)
        self.declare_parameter("max_extrapolation_sec", 0.25)
        self.declare_parameter("stats_period_sec", 5.0)
        # Drop body-frame XY returns inside this radius (m). 0.45 clears
        # ~0.58x0.44 footprint half-diagonal (~0.36) plus margin against
        # Mid360 self-hits that stamp ego cells occupied.
        self.declare_parameter("min_range_m", 0.45)

        self.input_topic = str(self.get_parameter("input_cloud_topic").value)
        self.output_topic = str(self.get_parameter("output_cloud_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.max_extrapolation_sec = float(
            self.get_parameter("max_extrapolation_sec").value
        )
        self.min_range_m = max(0.0, float(self.get_parameter("min_range_m").value))
        stats_period = float(self.get_parameter("stats_period_sec").value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        registered_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(PointCloud2, self.output_topic, registered_qos)
        self.create_subscription(
            PointCloud2, self.input_topic, self._on_cloud, sensor_qos
        )
        self._ok = 0
        self._drop = 0
        self._drop_tf = 0
        self._drop_stale = 0
        self._drop_xform = 0
        if stats_period > 0.0:
            self.create_timer(stats_period, self._on_stats)
        self.get_logger().info(
            "GT registered_scan relay '%s' -> '%s' (frame=%s, pub=RELIABLE/5, "
            "xyz-manual, min_range=%.3f m)"
            % (self.input_topic, self.output_topic, self.target_frame, self.min_range_m)
        )

    def _on_stats(self) -> None:
        self.get_logger().info(
            "registered_scan stats ok=%u drop=%u (tf=%u stale=%u xform=%u)"
            % (
                self._ok,
                self._drop,
                self._drop_tf,
                self._drop_stale,
                self._drop_xform,
            )
        )

    def _lookup_tf(self, source: str, stamp: Time):
        timeout = Duration(seconds=self.tf_timeout_sec)
        try:
            return self._tf_buffer.lookup_transform(
                self.target_frame, source, stamp, timeout=timeout
            )
        except Exception:
            return self._tf_buffer.lookup_transform(
                self.target_frame, source, Time(), timeout=timeout
            )

    def _on_cloud(self, msg: PointCloud2) -> None:
        source = msg.header.frame_id
        if not source:
            self._drop += 1
            return
        if source.lstrip("/") == self.target_frame.lstrip("/"):
            out = msg
            out.header.frame_id = self.target_frame
            self._pub.publish(out)
            self._ok += 1
            return
        stamp = Time.from_msg(msg.header.stamp)
        try:
            tf = self._lookup_tf(source, stamp)
        except Exception as exc:  # noqa: BLE001
            self._drop += 1
            self._drop_tf += 1
            if self._drop_tf % 50 == 1:
                self.get_logger().warn(
                    "TF %s<-%s failed (%s); dropped=%u ok=%u"
                    % (self.target_frame, source, exc, self._drop, self._ok)
                )
            return

        try:
            tf_time = Time.from_msg(tf.header.stamp)
            dt = abs((tf_time - stamp).nanoseconds) * 1e-9
            if dt > self.max_extrapolation_sec:
                self._drop += 1
                self._drop_stale += 1
                return
        except Exception:
            pass

        try:
            out = _transform_xyz_cloud(msg, tf, self.min_range_m)
        except Exception as exc:  # noqa: BLE001
            self._drop += 1
            self._drop_xform += 1
            if self._drop_xform % 20 == 1:
                self.get_logger().warn(
                    "xyz transform failed (%s); dropped=%u ok=%u"
                    % (exc, self._drop, self._ok)
                )
            return
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame
        self._pub.publish(out)
        self._ok += 1


def main() -> None:
    rclpy.init()
    node = GazeboGtRegisteredScanRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
